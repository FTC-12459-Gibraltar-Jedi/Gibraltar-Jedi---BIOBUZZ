package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.FieldConstants;
import org.firstinspires.ftc.teamcode.pedro.ShooterConstants;

/*
 * ╔══════════════════════════════════════════════════════════════════════════════════════════════╗
 * ║  SHOOTER                                                                                     ║
 * ║  Flywheel + turret (yaw servo) + hood (pitch servo). Used by TeleOp AND autonomous.          ║
 * ║                                                                                              ║
 * ║  How to use it:  set targets (aimAt, applyPreset, setRunning, ...) then call update()        ║
 * ║                  ONCE every loop — update() is what actually moves the hardware.             ║
 * ║                                                                                              ║
 * ║  Numbers to tune are in pedro/ShooterConstants.java, not here.                               ║
 * ╚══════════════════════════════════════════════════════════════════════════════════════════════╝
 */
public class Shooter {

    //==[HARDWARE]==================================================================================
    // Names must match the robot configuration on the Driver Station.

    private final DcMotorEx motor;        // "shooter"
    private final Servo yawServo;         // "yawServo"
    private final Servo pitchServo;       // "pitchServo"


    //==[STATE]=====================================================================================

    private double yaw = ShooterConstants.YAW_CENTER;       // turret servo target
    private double pitch = ShooterConstants.PITCH_MIN;      // hood servo target
    private double targetRpm = ShooterConstants.TARGET_RPM; // flywheel target
    private boolean running = false;                        // flywheel on/off
    private boolean loading = false;                        // hood parked for a hood swap (flywheel forced off)
    private boolean onBackHalf = false;                     // which half of the field we're aiming from

    // flywheel PIDF bookkeeping
    private final ElapsedTime timer = new ElapsedTime();
    private double lastError = 0.0;
    private double currentRpm = 0.0;
    private double power = 0.0;


    //==[SETUP]=====================================================================================

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "shooter");
        motor.setDirection(ShooterConstants.REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        yawServo = hardwareMap.get(Servo.class, "yawServo");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
    }


    //==[UPDATE — CALL EVERY LOOP]==================================================================

    public void update() {
        // ---- servos ----
        yaw = clamp(yaw, ShooterConstants.YAW_MIN, ShooterConstants.YAW_MAX);
        pitch = clamp(pitch, ShooterConstants.PITCH_MIN, ShooterConstants.PITCH_MAX);
        yawServo.setPosition(yaw);
        pitchServo.setPosition(loading ? ShooterConstants.HOOD_LOAD_POSITION : pitch);

        // ---- flywheel ----
        double dt = Math.max(0.001, timer.seconds());
        timer.reset();
        double currentTicksPerSec = motor.getVelocity();
        currentRpm = ticksToRpm(currentTicksPerSec);

        // never spin the flywheel while hands are in there swapping the hood
        if (running && !loading) {
            power = calculatePower(targetRpm, currentTicksPerSec, dt, lastError);
            lastError = rpmToTicksPerSec(targetRpm) - currentTicksPerSec;
        } else {
            power = 0.0;
            lastError = 0.0;
        }
        motor.setPower(power);
    }


    //==[AIMING]====================================================================================

    // Our alliance's goal on the half of the field the robot is on (split at FIELD_MIDLINE_Y).
    public Pose goalFor(Pose robot, boolean blueAlliance) {
        double line = FieldConstants.FIELD_MIDLINE_Y;
        double band = FieldConstants.MIDLINE_DEADBAND;
        if (robot.y() > line + band) onBackHalf = true;
        if (robot.y() < line - band) onBackHalf = false;
        return FieldConstants.goalFor(blueAlliance, onBackHalf);
    }

    // Points the turret at the goal and sets hood pitch + RPM from the distance (SHOT_TABLE).
    public void aimAt(Pose robot, Pose goal) {
        double distance = distanceToGoal(robot, goal);
        yaw = yawForPose(robot, goal);
        pitch = clamp(interpolateShotTable(distance, 1), ShooterConstants.PITCH_MIN, ShooterConstants.PITCH_MAX);
        targetRpm = interpolateShotTable(distance, 2);
    }

    // Loads a SHOT_TABLE row:  0 = close   1 = middle   2 = far
    public void applyPreset(int row) {
        pitch = ShooterConstants.SHOT_TABLE[row][1];
        targetRpm = ShooterConstants.SHOT_TABLE[row][2];
    }


    //==[MANUAL CONTROLS]===========================================================================

    public void adjustYaw(double delta)   { yaw += delta; }
    public void adjustPitch(double delta) { pitch += delta; }
    public void adjustRpm(double delta)   { targetRpm = Math.max(0, targetRpm + delta); }
    public void setRunning(boolean on)    { running = on; }

    // Hood swap mode. The normal pitch is remembered and restored when loading ends.
    public void toggleLoading()           { loading = !loading; }


    //==[STATUS]====================================================================================

    public boolean atSpeed() {
        return running && !loading && Math.abs(targetRpm - currentRpm) < ShooterConstants.RPM_TOLERANCE;
    }

    public boolean isRunning()     { return running; }
    public boolean isLoading()     { return loading; }
    public boolean isOnBackHalf()  { return onBackHalf; }
    public double getYaw()         { return yaw; }
    public double getPitch()       { return pitch; }
    public double getTargetRpm()   { return targetRpm; }
    public double getCurrentRpm()  { return currentRpm; }
    public double getPower()       { return power; }


    //==[IVY COMMANDS — FOR AUTONOMOUS]=============================================================

    // Keeps the turret aimed at our goal from wherever the robot is. Never finishes on its own.
    public Command aim(Follower follower, boolean blueAlliance) {
        return infinite(() -> {
            Pose robot = follower.pose();
            aimAt(robot, goalFor(robot, blueAlliance));
        }).requiring(this);
    }

    public Command spinUp()       { return instant(() -> running = true); }
    public Command waitForSpeed() { return waitUntil(this::atSpeed); }
    public Command stop()         { return instant(() -> running = false); }


    //==[MATH HELPERS]==============================================================================

    public static double distanceToGoal(Pose robot, Pose goal) {
        return Math.hypot(goal.x() - robot.x(), goal.y() - robot.y());
    }

    // Servo position that points the turret at the goal from the given robot pose.
    private static double yawForPose(Pose robot, Pose goal) {
        double fieldAngle = Math.atan2(goal.y() - robot.y(), goal.x() - robot.x());
        double turretDeg = Math.toDegrees(normalize(fieldAngle - robot.heading())) - ShooterConstants.YAW_OFFSET_DEG;
        turretDeg = Math.toDegrees(normalize(Math.toRadians(turretDeg)));
        double position = ShooterConstants.YAW_CENTER
                + ShooterConstants.YAW_DIRECTION * turretDeg / ShooterConstants.YAW_RANGE_DEG;
        return clamp(position, ShooterConstants.YAW_MIN, ShooterConstants.YAW_MAX);
    }

    // Blends a SHOT_TABLE column (1 = pitch, 2 = RPM) by distance; holds the end values outside the table.
    private static double interpolateShotTable(double distance, int column) {
        double[][] table = ShooterConstants.SHOT_TABLE;
        double[] first = table[0];
        double[] last = table[table.length - 1];
        if (distance <= first[0]) return first[column];
        if (distance >= last[0]) return last[column];

        for (int i = 0; i < table.length - 1; i++) {
            double[] a = table[i];
            double[] b = table[i + 1];
            if (distance <= b[0]) {
                double t = (distance - a[0]) / (b[0] - a[0]);
                return a[column] + t * (b[column] - a[column]);
            }
        }
        return last[column];
    }

    // PIDF power (-1 to 1) to reach targetRpm.
    private static double calculatePower(double targetRpm, double currentTicksPerSec, double dt, double lastError) {
        double targetTicksPerSec = rpmToTicksPerSec(targetRpm);
        double error = targetTicksPerSec - currentTicksPerSec;
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;

        double power = (ShooterConstants.kF * targetTicksPerSec)
                + (ShooterConstants.kP * error)
                + (ShooterConstants.kD * derivative);
        return clamp(power, -1.0, 1.0);
    }

    private static double rpmToTicksPerSec(double rpm) {
        return (rpm * ShooterConstants.TICKS_PER_REV) / 60.0;
    }

    private static double ticksToRpm(double ticksPerSec) {
        return (ticksPerSec * 60.0) / ShooterConstants.TICKS_PER_REV;
    }

    // wraps an angle to -PI..PI
    private static double normalize(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

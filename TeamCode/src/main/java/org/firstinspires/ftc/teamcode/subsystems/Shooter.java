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
 * ║  Flywheel + turret (yaw motor) + hood (pitch servo). Used by TeleOp AND autonomous.          ║
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
    private final DcMotorEx turretMotor;  // "turret"
    private final Servo pitchServo;       // "pitchServo"


    //==[STATE]=====================================================================================

    private double turretTargetDeg = 0;                     // turret target, 0 = straight forward, + = left
    private double turretAngleDeg = 0;                      // turret actual angle from the encoder
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

    // turret PID bookkeeping
    private double turretLastError = 0.0;
    private double turretPower = 0.0;


    //==[SETUP]=====================================================================================

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "shooter");
        motor.setDirection(ShooterConstants.REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // turret encoder is NOT reset here, so its zero carries over from auto into TeleOp
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
    }

    // Call with the turret pointed STRAIGHT FORWARD: makes that position 0°.
    public void zeroTurret() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretTargetDeg = 0;
        turretAngleDeg = 0;
        turretLastError = 0;
    }


    //==[UPDATE — CALL EVERY LOOP]==================================================================

    public void update() {
        double dt = Math.max(0.001, timer.seconds());
        timer.reset();

        // ---- hood ----
        pitch = clamp(pitch, ShooterConstants.PITCH_MIN, ShooterConstants.PITCH_MAX);
        pitchServo.setPosition(loading ? ShooterConstants.HOOD_LOAD_POSITION : pitch);

        // ---- turret (PID to the target angle, kept inside the software limits) ----
        turretTargetDeg = clamp(turretTargetDeg, ShooterConstants.TURRET_MIN_DEG, ShooterConstants.TURRET_MAX_DEG);
        turretAngleDeg = ShooterConstants.YAW_DIRECTION
                * turretMotor.getCurrentPosition() / ShooterConstants.TURRET_TICKS_PER_DEG;

        double turretError = turretTargetDeg - turretAngleDeg;
        double turretOut = ShooterConstants.TURRET_kP * turretError
                + ShooterConstants.TURRET_kD * (turretError - turretLastError) / dt;
        turretLastError = turretError;
        if (Math.abs(turretError) < ShooterConstants.TURRET_DEADBAND_DEG) turretOut = 0;

        // YAW_DIRECTION is applied to both the reading and the power, so flipping it only changes
        // which way is "left" — it can never make the turret run away
        turretPower = ShooterConstants.YAW_DIRECTION
                * clamp(turretOut, -ShooterConstants.TURRET_MAX_POWER, ShooterConstants.TURRET_MAX_POWER);
        turretMotor.setPower(turretPower);

        // ---- flywheel ----
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
        turretTargetDeg = turretAngleForPose(robot, goal);
        pitch = clamp(interpolateShotTable(distance, 1), ShooterConstants.PITCH_MIN, ShooterConstants.PITCH_MAX);
        targetRpm = interpolateShotTable(distance, 2);
    }

    // Loads a SHOT_TABLE row:  0 = close   1 = middle   2 = far
    public void applyPreset(int row) {
        pitch = ShooterConstants.SHOT_TABLE[row][1];
        targetRpm = ShooterConstants.SHOT_TABLE[row][2];
    }


    //==[MANUAL CONTROLS]===========================================================================

    public void adjustYaw(double degrees) { turretTargetDeg += degrees; }
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
    public double getTurretTargetDeg() { return turretTargetDeg; }
    public double getTurretAngleDeg()  { return turretAngleDeg; }
    public double getTurretPower()     { return turretPower; }
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

    // Turret angle (degrees, 0 = forward, + = left) that points at the goal from the given robot pose.
    // Goes up to 180° each way. When the goal crosses directly behind, the turret holds at the limit
    // until the goal is TURRET_SNAP_HYSTERESIS_DEG past it, then snaps around to the other side.
    private double turretAngleForPose(Pose robot, Pose goal) {
        double fieldAngle = Math.atan2(goal.y() - robot.y(), goal.x() - robot.x());
        double turretDeg = Math.toDegrees(normalize(fieldAngle - robot.heading())) - ShooterConstants.YAW_OFFSET_DEG;
        turretDeg = Math.toDegrees(normalize(Math.toRadians(turretDeg)));   // -180 .. 180

        double band = ShooterConstants.TURRET_SNAP_HYSTERESIS_DEG;
        boolean onLeftSide = turretTargetDeg > 90;     // + side, near the +180 limit
        boolean onRightSide = turretTargetDeg < -90;   // - side, near the -180 limit

        // goal just crossed behind: stay pinned on the side we're already on instead of snapping
        if (onLeftSide && turretDeg < -180 + band) return ShooterConstants.TURRET_MAX_DEG;
        if (onRightSide && turretDeg > 180 - band) return ShooterConstants.TURRET_MIN_DEG;

        return clamp(turretDeg, ShooterConstants.TURRET_MIN_DEG, ShooterConstants.TURRET_MAX_DEG);
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

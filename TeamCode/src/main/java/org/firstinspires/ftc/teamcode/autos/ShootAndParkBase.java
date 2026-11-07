package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Locale;

/*
 * ╔══════════════════════════════════════════════════════════════════════════════════════════════╗
 * ║  SHOOT AND PARK — SHARED CODE                                                                ║
 * ║  Not an auto by itself. BlueShootAndPark and RedShootAndPark use this.                       ║
 * ║  ➜ To change WHERE the robot starts/parks, edit those two files.                             ║
 * ║  ➜ To change the shooting timing/preset (both alliances), edit SETTINGS below.               ║
 * ╚══════════════════════════════════════════════════════════════════════════════════════════════╝
 *
 *  BEFORE INIT:  place the robot on the start pose with the turret pointed STRAIGHT FORWARD
 *                (INIT makes that turret position 0°).
 *
 *  ROUTINE
 *   1. Turret straight forward, hood + flywheel speed from SHOT_TABLE row SHOOT_PRESET_ROW
 *      (plus PITCH_ADJUST / RPM_ADJUST), flywheel starts spinning immediately
 *   2. INTAKE_DELAY_MS later, the intake starts feeding the preloads into the shooter
 *   3. Keep shooting until SHOOT_TIME_MS after the start
 *   4. Stop the intake and flywheel
 *   5. Drive to park: straight line off the wall, then a curve
 *   6. When stopped, hand the final position, alliance, and turret zero to TeleOp
 */
public abstract class ShootAndParkBase extends OpMode {

    //==[SETTINGS]==================================================================================

    private static final int SHOOT_PRESET_ROW = 2;          // SHOT_TABLE row: 0 = close, 1 = middle, 2 = far
    private static final double PITCH_ADJUST = 0.02;        // added to the preset's hood angle (auto only)
    private static final double RPM_ADJUST = -100;          // added to the preset's RPM (auto only)
    private static final double INTAKE_DELAY_MS = 500;      // flywheel spin-up time before the intake starts
    private static final double SHOOT_TIME_MS = 12000;      // total time from START until driving to park


    //==[SETUP FROM BLUE / RED FILE]================================================================

    private final boolean blueAlliance;
    private final Pose startPose;
    private final Pose driveOut;        // end of path 1
    private final Pose curveControl;    // bends path 2
    private final Pose parkPose;        // end of path 2

    protected ShootAndParkBase(boolean blueAlliance,
                               Pose startPose, Pose driveOut, Pose curveControl, Pose parkPose) {
        this.blueAlliance = blueAlliance;
        this.startPose = startPose;
        this.driveOut = driveOut;
        this.curveControl = curveControl;
        this.parkPose = parkPose;
    }


    //==[PATHS]=====================================================================================

    // path 1: short straight line off the wall
    private Path driveOutPath() {
        return line(startPose, driveOut).linear(startPose, driveOut);
    }

    // path 2: curve over to park, turning to the park heading on the way
    private Path curveToParkPath() {
        return curve(driveOut, curveControl, parkPose).linear(driveOut, parkPose);
    }


    //==[ROUTINE]===================================================================================

    private Command autoRoutine() {
        return sequential(
                // ---- shoot from the start spot ----
                instant(() -> {
                    shooter.setTurretTargetDeg(0);              // turret straight forward
                    shooter.applyPreset(SHOOT_PRESET_ROW);      // hood angle + flywheel speed
                    shooter.adjustPitch(PITCH_ADJUST);          // auto-only tweaks on top of the preset,
                    shooter.adjustRpm(RPM_ADJUST);              // so TeleOp's presets stay the same
                }),
                shooter.spinUp(),
                waitMs(INTAKE_DELAY_MS),
                intake.feed(),
                waitMs(SHOOT_TIME_MS - INTAKE_DELAY_MS),
                intake.stop(),
                shooter.stop(),

                // ---- park ----
                follow(follower, path(driveOutPath(), curveToParkPath()))
        );
    }


    //==[HARDWARE + STATE]==========================================================================

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private boolean started = false;


    // =============================================================================================
    //  OPMODE LIFECYCLE
    // =============================================================================================

    @Override
    public void init() {
        Scheduler.reset();   // the scheduler is static: clear anything left over from the last OpMode

        follower = Constants.create(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        // the turret must be set STRAIGHT FORWARD before INIT; this makes that 0° for auto and TeleOp
        shooter.zeroTurret();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("════════ " + (blueAlliance ? "BLUE" : "RED") + " SHOOT AND PARK ════════");
        telemetry.addLine("Robot on the start spot, turret pointed STRAIGHT FORWARD.");
    }

    @Override
    public void start() {
        started = true;
        follower.setPose(startPose);
        follower.update();

        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        shooter.update();

        Pose pose = follower.pose();
        telemetry.addData("Position", String.format(Locale.US, "x %.1f   y %.1f   heading %.0f°",
                pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Flywheel RPM", String.format(Locale.US, "%.0f / %.0f target",
                shooter.getCurrentRpm(), shooter.getTargetRpm()));
        telemetry.addData("Turret", String.format(Locale.US, "%.1f° / %.1f° target",
                shooter.getTurretAngleDeg(), shooter.getTurretTargetDeg()));
        telemetry.addData("Intake", String.format(Locale.US, "%.2f", intake.getPower()));
    }

    @Override
    public void stop() {
        // hand our final position, alliance, and turret zero to TeleOp (only if auto actually ran)
        if (started) {
            OpModeStorage.autonomousEndPose = follower.pose();
            OpModeStorage.blueAlliance = blueAlliance;
            OpModeStorage.turretZeroedByAuto = true;
        }
    }
}

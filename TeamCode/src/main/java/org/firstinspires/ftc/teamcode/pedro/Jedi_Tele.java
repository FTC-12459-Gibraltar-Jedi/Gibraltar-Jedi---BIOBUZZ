package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Locale;

/*
 * ╔══════════════════════════════════════════════════════════════════════════════════════════════╗
 * ║  JEDI TELEOP                                                                                 ║
 * ╚══════════════════════════════════════════════════════════════════════════════════════════════╝
 *
 *  INIT (either gamepad)
 *  ─────────────────────
 *   X / B ............ Blue / Red alliance (already set if an auto ran)
 *
 *  GAMEPAD 1 — DRIVER
 *  ──────────────────
 *   Left stick ....... Drive (field-centric: forward = away from your drivers)
 *   Right stick X .... Turn
 *   Left bumper ...... Hold for FULL speed (normal is 50%)
 *   Y ................ Reset heading (point robot away from drivers first)
 *   Dpad left ........ Relocalize: robot is in OUR LEFT corner
 *   Dpad right ....... Relocalize: robot is in OUR RIGHT corner
 *   A ................ Toggle flywheel on/off
 *   Right trigger .... Hold to run flywheel
 *
 *  GAMEPAD 2 — OPERATOR
 *  ────────────────────
 *   Left trigger ..... Hold for AUTO AIM (turret, hood, and RPM follow the robot; flywheel spins)
 *   Right stick X .... Manual turret
 *   Right stick Y .... Manual hood
 *   X ................ Turret forward reset (point turret STRAIGHT FORWARD first)
 *   Dpad down/left/up  Close / middle / far preset (SHOT_TABLE rows 0 / 1 / 2)
 *   Bumpers R / L .... +/- RPM (for tuning)
 *   Back ............. Hood load mode on/off (hood swap; flywheel forced off)
 *   A ................ Toggle flywheel on/off
 *   Right trigger .... Hold to run flywheel
 *
 *  Files to edit:  ShooterConstants (shooter numbers)   FieldConstants (field positions)
 */
@TeleOp
public class Jedi_Tele extends OpMode {

    //==[SETTINGS]==================================================================================

    private static final double SLOW_SPEED = 0.5;          // drive speed without left bumper
    private static final double RPM_STEP = 50;             // RPM per bumper press (gamepad 2)
    private static final double YAW_STICK_SPEED = 2.0;     // manual turret speed (degrees per loop)
    private static final double PITCH_STICK_SPEED = 0.005; // manual hood speed
    private static final double TRIGGER_THRESHOLD = 0.1;   // how far a trigger must be pulled to count


    //==[HARDWARE]==================================================================================

    private Follower follower;           // Pedro Pathing: drivetrain + odometry
    private Shooter shooter;             // flywheel, turret, hood
    private GoBildaPrismDriver prism;    // LEDs (null if "prism" isn't in the robot configuration)


    //==[STATE]=====================================================================================

    private boolean blueAlliance = OpModeStorage.blueAlliance;  // set by whichever auto ran last
    private boolean flywheelToggled = false;                     // A button toggle

    // filled in each loop, shown on telemetry
    private boolean autoAim = false;
    private boolean flywheelOn = false;
    private double goalDistance = 0;


    // =============================================================================================
    //  OPMODE LIFECYCLE   (init → init_loop → start → loop … → stop)
    // =============================================================================================

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        shooter = new Shooter(hardwareMap);

        // tryGet: run without LEDs instead of crashing if the Prism isn't configured
        prism = hardwareMap.tryGet(GoBildaPrismDriver.class, "prism");
        lightsOn();
    }

    @Override
    public void init_loop() {
        // only needed if no auto ran — otherwise the auto already picked the alliance
        if (gamepad1.x || gamepad2.x) blueAlliance = true;
        if (gamepad1.b || gamepad2.b) blueAlliance = false;

        telemetry.addLine("════════ JEDI TELEOP ════════");
        telemetry.addData("Alliance (X = blue, B = red)", blueAlliance ? "BLUE" : "RED");
        telemetry.addData("Start Pose", OpModeStorage.autonomousEndPose != null ? "FROM AUTO" : "DEFAULT");
        telemetry.addData("Prism LEDs", prism != null ? "CONNECTED" : "NOT CONFIGURED (\"prism\")");
    }

    @Override
    public void start() {
        if (OpModeStorage.autonomousEndPose != null) {
            // pick up exactly where auto left off, then forget it so a later TeleOp doesn't reuse it
            follower.setPose(OpModeStorage.autonomousEndPose);
            OpModeStorage.autonomousEndPose = null;
        } else {
            follower.setPose(FieldConstants.startPose(blueAlliance));
        }
        follower.update();
    }

    @Override
    public void loop() {
        updateDrive();

        Pose robotPose = follower.pose();
        updateTurretAndHood(robotPose);
        updateFlywheel();

        updateTelemetry(robotPose);
    }

    @Override
    public void stop() {
        lightsOff();
    }


    // =============================================================================================
    //  DRIVE   (gamepad 1)
    // =============================================================================================

    private void updateDrive() {
        double heading = follower.pose().heading();
        double driverHeading = FieldConstants.driverHeadingRadians(blueAlliance);

        // ---- Y: reset heading (robot must be pointed straight away from the drivers) ----
        if (gamepad1.yWasPressed()) {
            Pose current = follower.pose();
            follower.setHeading(driverHeading);
            heading = driverHeading;
            moveHoldTarget(new Pose(current.x(), current.y(), driverHeading));
        }

        // ---- dpad left/right: relocalize in one of our corners ----
        Pose corner = null;
        if (gamepad1.dpadLeftWasPressed())  corner = FieldConstants.leftCorner(blueAlliance);
        if (gamepad1.dpadRightWasPressed()) corner = FieldConstants.rightCorner(blueAlliance);
        if (corner != null) {
            follower.setPose(corner);
            heading = corner.heading();
            moveHoldTarget(corner);
        }

        // ---- sticks: field-centric drive, 50% speed unless left bumper is held ----
        double speed = gamepad1.left_bumper ? 1.0 : SLOW_SPEED;

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y * speed,
                -gamepad1.left_stick_x * speed,
                -gamepad1.right_stick_x * speed,
                heading,
                -driverHeading          // stick forward = away from our drivers
        );

        // holds position automatically when the sticks are released
        // (low input threshold because inputs are scaled down at 50% speed)
        ManualDrive.driveOrHold(follower, powers, 0.05, 0.2);
        follower.update();
    }

    // After changing the robot's position, auto hold's target must move too — otherwise the robot
    // drives/turns back to the old target, which is in the old coordinates.
    private void moveHoldTarget(Pose pose) {
        if (follower.holding()) {
            follower.hold(pose);
        }
    }


    // =============================================================================================
    //  TURRET + HOOD   (gamepad 2)
    // =============================================================================================

    private void updateTurretAndHood(Pose robotPose) {
        // ---- back: hood load mode ----
        if (gamepad2.backWasPressed()) shooter.toggleLoading();

        // ---- X: turret forward reset (turret must be pointed straight forward) ----
        if (gamepad2.xWasPressed()) shooter.zeroTurret();

        // ---- which goal: our color, on whichever half of the field we're on ----
        Pose goal = shooter.goalFor(robotPose, blueAlliance);
        goalDistance = Shooter.distanceToGoal(robotPose, goal);
        autoAim = gamepad2.left_trigger > TRIGGER_THRESHOLD;

        // ---- dpad: presets ----
        if (gamepad2.dpadDownWasPressed()) shooter.applyPreset(0);   // close
        if (gamepad2.dpadLeftWasPressed()) shooter.applyPreset(1);   // middle
        if (gamepad2.dpadUpWasPressed())   shooter.applyPreset(2);   // far

        // ---- aiming ----
        if (shooter.isLoading()) {
            // hood is parked for a swap: ignore aiming until back is pressed again
        } else if (autoAim) {
            shooter.aimAt(robotPose, goal);
        } else {
            shooter.adjustYaw(-gamepad2.right_stick_x * YAW_STICK_SPEED);   // stick right = turret right
            shooter.adjustPitch(gamepad2.right_stick_y * PITCH_STICK_SPEED);
            if (gamepad2.rightBumperWasPressed()) shooter.adjustRpm(RPM_STEP);
            if (gamepad2.leftBumperWasPressed())  shooter.adjustRpm(-RPM_STEP);
        }
    }


    // =============================================================================================
    //  FLYWHEEL   (both gamepads)
    // =============================================================================================

    private void updateFlywheel() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            flywheelToggled = !flywheelToggled;
        }

        flywheelOn = flywheelToggled
                || autoAim
                || gamepad1.right_trigger > TRIGGER_THRESHOLD
                || gamepad2.right_trigger > TRIGGER_THRESHOLD;

        shooter.setRunning(flywheelOn);
        shooter.update();
    }


    // =============================================================================================
    //  LIGHTS   (goBILDA Prism)
    // =============================================================================================

    // shows Artboard 0 (made with the "Prism Configurator" OpMode)
    private void lightsOn() {
        if (prism != null) prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
    }

    // turns the LEDs off to save battery; saved artboards are kept on the Prism
    private void lightsOff() {
        if (prism != null) prism.clearAllAnimations();
    }


    // =============================================================================================
    //  TELEMETRY
    // =============================================================================================

    private void updateTelemetry(Pose robotPose) {
        String alliance = blueAlliance ? "BLUE" : "RED";

        if (shooter.isLoading()) {
            telemetry.addLine("⚠ HOOD LOADING — press BACK to exit");
            telemetry.addLine();
        }

        telemetry.addLine("──────── ROBOT ────────");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Position", String.format(Locale.US, "x %.1f   y %.1f   heading %.0f°",
                robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading())));
        telemetry.addData("Drive Speed", gamepad1.left_bumper ? "FULL" : "50%");

        telemetry.addLine("──────── AIM ────────");
        telemetry.addData("Auto Aim", autoAim ? "ON" : "off");
        telemetry.addData("Aiming At", alliance + (shooter.isOnFarHalf() ? " FAR" : " CLOSE") + " goal");
        telemetry.addData("Goal Distance", String.format(Locale.US, "%.1f in", goalDistance));
        telemetry.addData("Turret", String.format(Locale.US, "%.1f° / %.1f° target   (power %.2f)",
                shooter.getTurretAngleDeg(), shooter.getTurretTargetDeg(), shooter.getTurretPower()));
        telemetry.addData("Hood (pitch)", String.format(Locale.US, "%.3f", shooter.getPitch()));

        telemetry.addLine("──────── FLYWHEEL ────────");
        telemetry.addData("Running", flywheelOn && !shooter.isLoading() ? "YES" : "no");
        telemetry.addData("RPM", String.format(Locale.US, "%.0f / %.0f target",
                shooter.getCurrentRpm(), shooter.getTargetRpm()));
        telemetry.addData("Ready to Shoot", shooter.atSpeed() ? "YES" : "no");
        telemetry.addData("Motor Power", String.format(Locale.US, "%.2f", shooter.getPower()));

        telemetry.addLine("──────── TUNING ────────");
        // copy this line straight into SHOT_TABLE once the shot scores from here
        telemetry.addLine(String.format(Locale.US, "SHOT_TABLE row: { %.1f, %.3f, %.0f },",
                goalDistance, shooter.getPitch(), shooter.getTargetRpm()));
    }
}

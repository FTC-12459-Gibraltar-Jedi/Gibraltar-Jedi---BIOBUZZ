package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Locale;

/*
 * ╔══════════════════════════════════════════════════════════════════════════════════════════════╗
 * ║  JEDI AUTO — shoot preloads, then park                                                       ║
 * ╚══════════════════════════════════════════════════════════════════════════════════════════════╝
 *
 *  INIT:   X = blue   B = red   (either gamepad)
 *
 *  ROUTINE
 *   1. Spin up the flywheel and drive to the shooting spot (turret aims the whole time)
 *   2. Wait for the flywheel to reach speed, then shoot the preloads
 *   3. Stop the flywheel and drive to park
 *   4. When stopped, hand the final position + alliance to TeleOp
 *
 *  Written with Ivy commands: each step in sequential(...) runs after the one before finishes.
 */
@Autonomous
public class Jedi_Auto extends OpMode {

    //==[SETTINGS]==================================================================================

    private static final double SPIN_UP_TIMEOUT_MS = 2000;  // max wait for the flywheel before shooting anyway
    private static final double FEED_MS = 2000;             // time given to feed the preloads


    //==[POSES]=====================================================================================
    // Written for RED (red drivers at x = 0) in Pedro coordinates (inches, heading in degrees).
    // Blue uses the same numbers mirrored across the middle of the field (x = 72).
    // ⚠ MEASURE THESE ON YOUR FIELD.

    private void buildPoses() {
        PoseFactory p = PoseFactory.degrees();
        if (blueAlliance) p = p.mirrorX(72);

        startPose = p.of(24, 12, 90);
        shootPose = p.of(48, 48, 90);
        parkPose  = p.of(36, 72, 90);
    }


    //==[PATHS]=====================================================================================

    private Path startToShoot() {
        return line(startPose, shootPose).linear(startPose, shootPose);
    }

    private Path shootToPark() {
        return line(shootPose, parkPose).linear(shootPose, parkPose);
    }


    //==[ROUTINE]===================================================================================

    private Command autoRoutine() {
        return sequential(
                shooter.spinUp(),
                race(                                           // ends when the drive + shoot finishes
                        shooter.aim(follower, blueAlliance),    // keep aiming the whole time
                        sequential(
                                follow(follower, startToShoot()),
                                shootPreloads()
                        )
                ),
                shooter.stop(),
                follow(follower, shootToPark())
        );
    }

    private Command shootPreloads() {
        return sequential(
                shooter.waitForSpeed().raceWith(waitMs(SPIN_UP_TIMEOUT_MS)),
                // TODO: run the feeder/intake here to push the preloads into the flywheel
                waitMs(FEED_MS)
        );
    }


    //==[HARDWARE + STATE]==========================================================================

    private Follower follower;
    private Shooter shooter;

    private boolean blueAlliance = OpModeStorage.blueAlliance;

    // built in start() once the alliance is locked in
    private Pose startPose;
    private Pose shootPose;
    private Pose parkPose;


    // =============================================================================================
    //  OPMODE LIFECYCLE
    // =============================================================================================

    @Override
    public void init() {
        Scheduler.reset();   // the scheduler is static: clear anything left over from the last OpMode

        follower = Constants.create(hardwareMap);
        shooter = new Shooter(hardwareMap);

        // the turret must be set STRAIGHT FORWARD before INIT; this makes that 0° for auto and TeleOp
        shooter.zeroTurret();
    }

    @Override
    public void init_loop() {
        if (gamepad1.x || gamepad2.x) blueAlliance = true;
        if (gamepad1.b || gamepad2.b) blueAlliance = false;

        telemetry.addLine("════════ JEDI AUTO ════════");
        telemetry.addData("Alliance (X = blue, B = red)", blueAlliance ? "BLUE" : "RED");
    }

    @Override
    public void start() {
        buildPoses();
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
    }

    @Override
    public void stop() {
        // hand our final position and alliance to TeleOp (position only if auto actually ran)
        if (startPose != null) {
            OpModeStorage.autonomousEndPose = follower.pose();
        }
        OpModeStorage.blueAlliance = blueAlliance;
    }
}

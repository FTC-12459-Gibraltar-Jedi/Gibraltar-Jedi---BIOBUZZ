package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.Locale;

/*
 * ╔══════════════════════════════════════════════════════════════════════════════════════════════╗
 * ║  LEAVE AND PARK — SHARED CODE                                                                ║
 * ║  Not an auto by itself. BlueLeaveAndPark and RedLeaveAndPark use this.                       ║
 * ║  ➜ To change WHERE the robot starts/parks, edit those two files, not this one.               ║
 * ╚══════════════════════════════════════════════════════════════════════════════════════════════╝
 *
 *  INIT MENU (gamepad 1)
 *   Dpad up / down ...... pick the row (Start or End)
 *   Dpad left / right ... change the option on that row
 *
 *  Drives in a straight line from the chosen start to the chosen park spot, then hands the
 *  final position and alliance to TeleOp.
 */
public abstract class LeaveAndParkBase extends OpMode {

    //==[MENU OPTIONS]==============================================================================
    // Names shown on the Driver Station, in the same order as the poses passed in below.

    private static final String[] START_NAMES = { "Left Close", "Right Close", "Right Far" };
    private static final String[] PARK_NAMES  = { "Park Left", "Park Right" };


    //==[SETUP FROM BLUE / RED FILE]================================================================

    private final boolean blueAlliance;
    private final Pose[] startPoses;
    private final Pose[] parkPoses;

    protected LeaveAndParkBase(boolean blueAlliance,
                               Pose leftCloseStart, Pose rightCloseStart, Pose rightFarStart,
                               Pose parkLeft, Pose parkRight) {
        this.blueAlliance = blueAlliance;
        this.startPoses = new Pose[] { leftCloseStart, rightCloseStart, rightFarStart };
        this.parkPoses = new Pose[] { parkLeft, parkRight };
    }


    //==[STATE]=====================================================================================

    private Follower follower;

    private int start = 0;                  // chosen start option
    private int park = 0;                   // chosen park option
    private boolean editingStart = true;    // which menu row the cursor is on
    private boolean started = false;


    //==[PATH]======================================================================================

    private Path leaveAndPark() {
        Pose from = startPoses[start];
        Pose to = parkPoses[park];
        return line(from, to).linear(from, to);
    }


    // =============================================================================================
    //  OPMODE LIFECYCLE
    // =============================================================================================

    @Override
    public void init() {
        Scheduler.reset();   // the scheduler is static: clear anything left over from the last OpMode
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void init_loop() {
        // ---- menu controls ----
        if (gamepad1.dpadUpWasPressed() || gamepad1.dpadDownWasPressed()) editingStart = !editingStart;

        int step = 0;
        if (gamepad1.dpadRightWasPressed()) step = 1;
        if (gamepad1.dpadLeftWasPressed())  step = -1;
        if (step != 0) {
            if (editingStart) {
                start = Math.floorMod(start + step, startPoses.length);
            } else {
                park = Math.floorMod(park + step, parkPoses.length);
            }
        }

        // ---- menu display ----
        telemetry.addLine("════════ " + (blueAlliance ? "BLUE" : "RED") + " LEAVE AND PARK ════════");
        telemetry.addLine("Dpad up/down = pick row, left/right = change");
        telemetry.addLine();
        telemetry.addLine((editingStart ? " ▶ " : "    ") + "Start:  " + START_NAMES[start]);
        telemetry.addLine((editingStart ? "    " : " ▶ ") + "End:    " + PARK_NAMES[park]);
    }

    @Override
    public void start() {
        started = true;
        follower.setPose(startPoses[start]);
        follower.update();

        schedule(follow(follower, leaveAndPark()));
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        Pose pose = follower.pose();
        telemetry.addData("Route", START_NAMES[start] + "  →  " + PARK_NAMES[park]);
        telemetry.addData("Position", String.format(Locale.US, "x %.1f   y %.1f   heading %.0f°",
                pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        telemetry.addData("Follower Mode", follower.mode());
    }

    @Override
    public void stop() {
        // hand our position and alliance to TeleOp so it's ready to go (only if auto actually ran)
        if (started) {
            OpModeStorage.autonomousEndPose = follower.pose();
            OpModeStorage.blueAlliance = blueAlliance;
        }
    }
}

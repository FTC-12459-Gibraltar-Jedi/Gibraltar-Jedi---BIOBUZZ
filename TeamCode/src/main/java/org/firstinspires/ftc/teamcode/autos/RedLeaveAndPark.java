package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * ╔══════════════════════════════════════════════════════════════════════════════════════════════╗
 * ║  RED LEAVE AND PARK                                                                          ║
 * ║  ➜ Just edit the poses below. The menu and driving code is in LeaveAndParkBase.              ║
 * ╚══════════════════════════════════════════════════════════════════════════════════════════════╝
 *
 *  p.of(x, y, heading)   inches, heading in degrees, Pedro coordinates (see FieldConstants)
 *  ⚠ MEASURE THESE ON YOUR FIELD.
 */
@Autonomous(name = "Red Leave and Park", group = "Red")
public class RedLeaveAndPark extends LeaveAndParkBase {

    private static final PoseFactory p = PoseFactory.degrees();

    //==[START POSES]===============================================================================

    // Red is on the x = 0 side of the field.
    private static final Pose LEFT_CLOSE_START  = p.of(8.9697, 133.5506, 0);      // from visualizer
    private static final Pose RIGHT_CLOSE_START = p.of(8.5607, 83.1122, 0);       // from visualizer
    private static final Pose RIGHT_FAR_START   = p.of(8.8333, 33.7645, 0);       // from visualizer

    //==[WAYPOINTS]=================================================================================
    // Optional point to drive to first, before heading to park. null = drive straight to park.

    private static final Pose LEFT_CLOSE_WAYPOINT  = null;
    private static final Pose RIGHT_CLOSE_WAYPOINT = null;
    private static final Pose RIGHT_FAR_WAYPOINT   = p.of(30.0992, 33.8189, 0);   // from visualizer

    //==[PARK POSES]================================================================================

    private static final Pose PARK_LEFT  = p.of(13.7408, 123.1084, 0);            // from visualizer
    private static final Pose PARK_RIGHT = p.of(13.7065, 92.875, 0);              // from visualizer


    public RedLeaveAndPark() {
        super(false,
                LEFT_CLOSE_START,  LEFT_CLOSE_WAYPOINT,
                RIGHT_CLOSE_START, RIGHT_CLOSE_WAYPOINT,
                RIGHT_FAR_START,   RIGHT_FAR_WAYPOINT,
                PARK_LEFT, PARK_RIGHT);
    }
}

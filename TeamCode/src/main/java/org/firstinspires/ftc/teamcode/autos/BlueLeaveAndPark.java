package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * ╔══════════════════════════════════════════════════════════════════════════════════════════════╗
 * ║  BLUE LEAVE AND PARK                                                                         ║
 * ║  ➜ Just edit the poses below. The menu and driving code is in LeaveAndParkBase.              ║
 * ╚══════════════════════════════════════════════════════════════════════════════════════════════╝
 *
 *  p.of(x, y, heading)   inches, heading in degrees, Pedro coordinates (see FieldConstants)
 *  ⚠ MEASURE THESE ON YOUR FIELD.
 */
@Autonomous(name = "Blue Leave and Park", group = "Blue")
public class BlueLeaveAndPark extends LeaveAndParkBase {

    private static final PoseFactory p = PoseFactory.degrees();

    //==[START POSES]===============================================================================

    // Blue is on the x = 144 side of the field.
    // These are RED's poses rotated 180° around the field center:  x → 144 − x,  y → 144 − y,
    // heading + 180°.  That keeps left/right the same from each drive team's point of view.
    private static final Pose LEFT_CLOSE_START  = p.of(135.0303, 10.4494, 180);   // mirrored from red
    private static final Pose RIGHT_CLOSE_START = p.of(135.4393, 60.8878, 180);   // mirrored from red
    private static final Pose RIGHT_FAR_START   = p.of(135.1667, 110.2355, 180);  // mirrored from red

    //==[WAYPOINTS]=================================================================================
    // Optional point to drive to first, before heading to park. null = drive straight to park.

    private static final Pose LEFT_CLOSE_WAYPOINT  = null;
    private static final Pose RIGHT_CLOSE_WAYPOINT = null;
    private static final Pose RIGHT_FAR_WAYPOINT   = p.of(113.9008, 110.1811, 180); // mirrored from red

    //==[PARK POSES]================================================================================

    private static final Pose PARK_LEFT  = p.of(130.2592, 20.8916, 180);          // mirrored from red
    private static final Pose PARK_RIGHT = p.of(130.2935, 51.125, 180);           // mirrored from red


    public BlueLeaveAndPark() {
        super(true,
                LEFT_CLOSE_START,  LEFT_CLOSE_WAYPOINT,
                RIGHT_CLOSE_START, RIGHT_CLOSE_WAYPOINT,
                RIGHT_FAR_START,   RIGHT_FAR_WAYPOINT,
                PARK_LEFT, PARK_RIGHT);
    }
}

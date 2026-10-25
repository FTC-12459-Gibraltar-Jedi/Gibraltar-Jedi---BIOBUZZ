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

    private static final Pose LEFT_CLOSE_START  = p.of(24, 12, 90);
    private static final Pose RIGHT_CLOSE_START = p.of(48, 12, 90);
    private static final Pose RIGHT_FAR_START   = p.of(48, 132, 270);

    //==[PARK POSES]================================================================================

    private static final Pose PARK_LEFT  = p.of(24, 72, 90);
    private static final Pose PARK_RIGHT = p.of(48, 72, 90);


    public BlueLeaveAndPark() {
        super(true, LEFT_CLOSE_START, RIGHT_CLOSE_START, RIGHT_FAR_START, PARK_LEFT, PARK_RIGHT);
    }
}

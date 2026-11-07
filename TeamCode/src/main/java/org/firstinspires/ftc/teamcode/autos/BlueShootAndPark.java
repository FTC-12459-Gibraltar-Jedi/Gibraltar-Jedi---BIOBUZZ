package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * ╔══════════════════════════════════════════════════════════════════════════════════════════════╗
 * ║  BLUE SHOOT AND PARK — shoot the preloads right away, then drive to park                     ║
 * ║  ➜ Just edit the poses below. The routine and its timing are in ShootAndParkBase.            ║
 * ╚══════════════════════════════════════════════════════════════════════════════════════════════╝
 *
 *  p.of(x, y, heading)   inches, heading in degrees, Pedro coordinates (from the visualizer)
 */
@Autonomous(name = "Blue Shoot and Park", group = "Blue")
public class BlueShootAndPark extends ShootAndParkBase {

    private static final PoseFactory p = PoseFactory.degrees();

    //==[POSES — FROM THE PEDRO VISUALIZER]=========================================================

    private static final Pose START_POSE    = p.of(57.3632, 7.8637, 90);
    private static final Pose DRIVE_OUT     = p.of(57.4995, 16.6426, 90);   // end of path 1 (straight line)
    private static final Pose CURVE_CONTROL = p.of(17.1557, 34.5846, 0);    // bends path 2
    private static final Pose PARK_POSE     = p.of(10.886, 99.9619, 0);     // end of path 2 (curve)


    public BlueShootAndPark() {
        super(true, START_POSE, DRIVE_OUT, CURVE_CONTROL, PARK_POSE);
    }
}

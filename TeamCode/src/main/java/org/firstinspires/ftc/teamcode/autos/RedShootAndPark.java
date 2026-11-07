package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * ╔══════════════════════════════════════════════════════════════════════════════════════════════╗
 * ║  RED SHOOT AND PARK — shoot the preloads right away, then drive to park                      ║
 * ║  ➜ Just edit the poses below. The routine and its timing are in ShootAndParkBase.            ║
 * ╚══════════════════════════════════════════════════════════════════════════════════════════════╝
 *
 *  p.of(x, y, heading)   inches, heading in degrees, Pedro coordinates
 *  These are BLUE's poses rotated 180° around the field center:  x → 144 − x,  y → 144 − y,
 *  heading + 180°  (same mirroring as Leave and Park).
 */
@Autonomous(name = "Red Shoot and Park", group = "Red")
public class RedShootAndPark extends ShootAndParkBase {

    private static final PoseFactory p = PoseFactory.degrees();

    //==[POSES — MIRRORED FROM BLUE]================================================================

    private static final Pose START_POSE    = p.of(86.6368, 136.1363, 270);
    private static final Pose DRIVE_OUT     = p.of(86.5005, 127.3574, 270);  // end of path 1 (straight line)
    private static final Pose CURVE_CONTROL = p.of(126.8443, 109.4154, 180); // bends path 2
    private static final Pose PARK_POSE     = p.of(133.114, 44.0381, 180);   // end of path 2 (curve)


    public RedShootAndPark() {
        super(false, START_POSE, DRIVE_OUT, CURVE_CONTROL, PARK_POSE);
    }
}

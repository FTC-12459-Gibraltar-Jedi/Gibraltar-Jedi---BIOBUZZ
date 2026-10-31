package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;

/*
 * ╔══════════════════════════════════════════════════════════════════════════════════════════════╗
 * ║  FIELD CONSTANTS                                                                             ║
 * ║  Every position on the field that TeleOp uses: start spots, corners, and goals.              ║
 * ║  (Auto start/park spots live in the auto files themselves.)                                  ║
 * ╚══════════════════════════════════════════════════════════════════════════════════════════════╝
 *
 *  HOW THE COORDINATES WORK (Pedro Pathing)
 *  ─────────────────────────────────────────
 *  • Stand on the AUDIENCE side looking at the field.
 *  • (0, 0) is the corner on your LEFT.  X goes to the RIGHT, Y goes AWAY from you.
 *  • Everything is in INCHES.  The field is about 144 x 144.
 *  • Heading:  0° = facing right (+X)    90° = facing away from the audience (+Y)
 *              180° = facing left        270° = facing the audience
 *  • A Pose is the CENTER of the robot:  new Pose(x, y, Math.toRadians(headingDegrees))
 *
 *                 Y = 144  (back wall)
 *          ┌──────────────────────────────┐
 *          │                              │
 *   RED    │                              │   BLUE
 *  drivers │  → faces 0°        180° ←    │  drivers
 *  (x = 0) │                              │ (x = 144)
 *          │                              │
 *          └──────────────────────────────┘
 *        (0,0)     Y = 0  (audience)
 *
 *  ⚠ MEASURE ALL OF THESE ON YOUR REAL FIELD — the numbers below are placeholders.
 */
public class FieldConstants {

    //==[TELEOP START POSES]========================================================================
    // Where TeleOp thinks the robot is when it starts WITHOUT running an auto first.
    // (After an auto, TeleOp uses the auto's final position instead.)

    public static final Pose BLUE_START_POSE = new Pose(120, 72, Math.toRadians(180));
    public static final Pose RED_START_POSE  = new Pose(24, 72, Math.toRadians(0));


    //==[DRIVER DIRECTION]==========================================================================
    // The field heading (degrees) that points straight AWAY from each alliance's drivers.
    //  • Pushing the left stick forward drives this way.
    //  • Gamepad 1 Y resets the robot's heading to this.

    public static final double BLUE_DRIVER_HEADING_DEG = 180;   // blue drivers at x = 144, facing -X
    public static final double RED_DRIVER_HEADING_DEG  = 0;     // red drivers at x = 0, facing +X


    //==[RELOCALIZATION CORNERS]====================================================================
    // Gamepad 1 dpad LEFT / RIGHT in TeleOp.
    //  1. Push the robot flat into the corner, facing away from your drivers.
    //  2. Press the button.
    // Left/right are as seen by YOUR drivers. For an 18" robot the center is 9" from each wall.

    // The heading must be the way the robot REALLY faces in the corner (away from your drivers),
    // so it should match that alliance's DRIVER HEADING above.
    public static final Pose BLUE_LEFT_CORNER  = new Pose(133.5, 8.5, Math.toRadians(180));
    public static final Pose BLUE_RIGHT_CORNER = new Pose(133.5, 133, Math.toRadians(180));

    public static final Pose RED_LEFT_CORNER   = new Pose(8.5, 133.5, Math.toRadians(0));
    public static final Pose RED_RIGHT_CORNER  = new Pose(8.5, 8.5, Math.toRadians(0));


    //==[GOALS]=====================================================================================
    // The field is split in half at y = FIELD_MIDLINE_Y.
    // Auto aim shoots at OUR alliance's goal on whichever half the robot is on:
    //   CLOSE goal = robot is below the midline (y < 72)      FAR goal = robot is above it (y > 72)

    public static final Pose BLUE_GOAL_CLOSE = new Pose(84, 50, 0);
    public static final Pose BLUE_GOAL_FAR   = new Pose(84, 90, 0);

    public static final Pose RED_GOAL_CLOSE  = new Pose(58, 50, 0);
    public static final Pose RED_GOAL_FAR    = new Pose(58, 90, 0);

    public static final double FIELD_MIDLINE_Y  = 72;
    public static final double MIDLINE_DEADBAND = 2;   // inches past the line before switching, so it doesn't flicker


    //==[HELPERS]===================================================================================

    public static Pose goalFor(boolean blueAlliance, boolean farHalf) {
        if (blueAlliance) return farHalf ? BLUE_GOAL_FAR : BLUE_GOAL_CLOSE;
        return farHalf ? RED_GOAL_FAR : RED_GOAL_CLOSE;
    }

    public static Pose startPose(boolean blueAlliance) {
        return blueAlliance ? BLUE_START_POSE : RED_START_POSE;
    }

    public static double driverHeadingRadians(boolean blueAlliance) {
        return Math.toRadians(blueAlliance ? BLUE_DRIVER_HEADING_DEG : RED_DRIVER_HEADING_DEG);
    }

    public static Pose leftCorner(boolean blueAlliance) {
        return blueAlliance ? BLUE_LEFT_CORNER : RED_LEFT_CORNER;
    }

    public static Pose rightCorner(boolean blueAlliance) {
        return blueAlliance ? BLUE_RIGHT_CORNER : RED_RIGHT_CORNER;
    }
}

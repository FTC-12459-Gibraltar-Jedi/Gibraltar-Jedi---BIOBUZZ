package org.firstinspires.ftc.teamcode.pedro;

/*
 * ╔══════════════════════════════════════════════════════════════════════════════════════════════╗
 * ║  SHOOTER CONSTANTS                                                                           ║
 * ║  Every number you tune for the flywheel, turret (yaw), and hood (pitch).                     ║
 * ║  Only numbers live here — the math that uses them is in subsystems/Shooter.java.             ║
 * ╚══════════════════════════════════════════════════════════════════════════════════════════════╝
 */
public class ShooterConstants {

    //==[FLYWHEEL MOTOR]============================================================================

    public static boolean REVERSED = true;                // flip if the flywheel spins backwards
    public static final double TICKS_PER_REV = 28.0;      // encoder ticks per motor revolution

    public static final double TARGET_RPM = 3100;         // starting speed, until a preset / auto aim changes it
    public static final double RPM_TOLERANCE = 100;       // within this many RPM of target = "ready to shoot"


    //==[FLYWHEEL PIDF]=============================================================================
    // Gentle tuning that stops the flywheel from oscillating.

    public static final double kP = 0.00482;
    public static final double kI = 0.00000;
    public static final double kD = 0.00000;              // keep at 0: the encoder is too noisy for D
    public static final double kF = 0.000421;             // raise if the flywheel can't reach high RPM


    //==[TURRET YAW SERVO]==========================================================================

    public static final double YAW_MIN = 0.0;             // servo limits
    public static final double YAW_MAX = 1.0;
    public static final double YAW_CENTER = 0.5;          // servo position when the turret points straight ahead
    public static final double YAW_RANGE_DEG = 300.0;     // degrees the turret turns over the full 0 → 1 servo travel
    public static final double YAW_DIRECTION = 1.0;       // flip to -1.0 if the turret aims the wrong way
    public static final double YAW_OFFSET_DEG = 0.0;      // 180 if the shooter faces the back of the robot


    //==[HOOD PITCH SERVO]==========================================================================

    public static final double PITCH_MIN = 0.02;          // hood at its lowest useful angle
    public static final double PITCH_MAX = 0.35;          // hood at its highest useful angle

    // Gamepad 2 BACK parks the hood gear here so a replacement hood can be dropped in.
    // Allowed to be outside PITCH_MIN / PITCH_MAX.
    public static final double HOOD_LOAD_POSITION = 0.47;


    //==[SHOT TABLE]================================================================================
    // Each row:  { distance to goal (inches),  pitch servo position,  flywheel RPM }
    //  • Keep rows sorted closest → furthest.
    //  • Auto aim blends between rows based on distance.
    //  • Gamepad 2 dpad presets use:  DOWN = row 0   LEFT = row 1   UP = row 2
    //
    // To tune a row: park, load the preset, adjust pitch (right stick) and RPM (bumpers) until it
    // scores, then copy the "SHOT_TABLE row:" line from telemetry into here.

    public static final double[][] SHOT_TABLE = {
            //  distance   pitch   RPM
            {    36.0,     0.05,   2600 },   // row 0 — close
            {    72.0,     0.15,   3100 },   // row 1 — middle
            {   120.0,     0.28,   3700 },   // row 2 — far
    };
}

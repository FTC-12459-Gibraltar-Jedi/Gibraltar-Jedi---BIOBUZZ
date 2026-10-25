package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/*
 * ╔══════════════════════════════════════════════════════════════════════════════════════════════╗
 * ║  OPMODE STORAGE                                                                              ║
 * ║  Hands values from autonomous to TeleOp.                                                     ║
 * ╚══════════════════════════════════════════════════════════════════════════════════════════════╝
 *
 *  Autos write these when they stop; TeleOp reads them when it starts.
 *  They survive between OpModes as long as the Robot Controller app isn't restarted
 *  (don't power-cycle the robot between auto and TeleOp).
 */
public class OpModeStorage {

    // where the robot ended auto; null = no auto has run since the last TeleOp
    public static Pose autonomousEndPose = null;

    // alliance picked in auto, so TeleOp starts already set up
    public static boolean blueAlliance = true;
}

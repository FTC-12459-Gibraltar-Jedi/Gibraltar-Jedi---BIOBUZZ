package org.firstinspires.ftc.teamcode.pedro;

public class ShooterConstants {

    // Shooter Motor Direction - Set to true so motor spins forward
    public static boolean REVERSED = true;

    // Shooter Motor Encoder Configuration
    public static final double TICKS_PER_REV = 28.0;

    // Gentle PD/F Tuning set (eliminates flywheel oscillation)
    public static final double kP = 0.00482;
    public static final double kI = 0.00000;
    public static final double kD = 0.00000; // Set D to 0 to eliminate encoder derivative noise/oscillation
    public static final double kF = 0.000421;

    // Target Speed in RPM
    public static final double TARGET_RPM = 2800.0;

    // Helper Calculations & Conversions
    public static double getTargetTicksPerSec() {
        return (TARGET_RPM * TICKS_PER_REV) / 60.0;
    }

    public static double ticksToRPM(double ticksPerSec) {
        return (ticksPerSec * 60.0) / TICKS_PER_REV;
    }

    /**
     * Calculates the PIDF power output (-1.0 to 1.0) required to reach TARGET_RPM.
     */
    public static double calculatePower(double currentTicksPerSec, double dt, double lastError) {
        double targetTicksPerSec = getTargetTicksPerSec();
        double error = targetTicksPerSec - currentTicksPerSec;
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;

        double power = (kF * targetTicksPerSec) + (kP * error) + (kD * derivative);
        return Math.max(-1.0, Math.min(1.0, power));
    }
}






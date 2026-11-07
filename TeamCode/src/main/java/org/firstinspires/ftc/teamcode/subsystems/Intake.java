package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * ╔══════════════════════════════════════════════════════════════════════════════════════════════╗
 * ║  INTAKE                                                                                      ║
 * ║  One motor. Positive power pulls balls IN (and feeds them up into the shooter),              ║
 * ║  negative power spits them OUT. Used by TeleOp AND autonomous.                               ║
 * ╚══════════════════════════════════════════════════════════════════════════════════════════════╝
 */
public class Intake {

    //==[SETTINGS]==================================================================================

    public static final boolean REVERSED = true;    // flip if "in" spins the wrong way
    public static final double FEED_POWER = 1.0;    // power used to feed balls into the shooter in auto


    //==[HARDWARE]==================================================================================

    private final DcMotor motor;    // "intake"

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "intake");
        motor.setDirection(REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }


    //==[CONTROL]===================================================================================

    // -1 (out) … 0 (stopped) … +1 (in)
    public void setPower(double power) { motor.setPower(power); }
    public double getPower()           { return motor.getPower(); }


    //==[IVY COMMANDS — FOR AUTONOMOUS]=============================================================

    public Command feed() { return instant(() -> motor.setPower(FEED_POWER)); }
    public Command stop() { return instant(() -> motor.setPower(0)); }
}

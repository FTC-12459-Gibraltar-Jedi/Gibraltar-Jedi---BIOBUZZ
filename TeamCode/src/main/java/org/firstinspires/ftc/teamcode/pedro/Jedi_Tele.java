package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.drivetrain.DrivePowers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

//==[STATIC]========================================================================================
@TeleOp
public class Jedi_Tele extends OpMode {

    //--Turret & Shooter--
    private DcMotorEx shooter = null;
    private Servo yawServo = null;
    private Servo pitchServo = null;

    private double yawAngle = 0.5;
    private double pitchAngle = 0.02;

    // Shooter PID state
    private boolean shooterActive = false;
    private boolean lastGamepadA = false;
    private double lastError = 0.0;
    private double lastTime = 0.0;

    private Follower follower;

//==[INIT]==========================================================================================
    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(ShooterConstants.REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        yawServo = hardwareMap.get(Servo.class, "yawServo");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        follower = Constants.create(hardwareMap);
    }

//==[LOOP]==========================================================================================
    @Override
    public void loop() {

        //----Drive Controls----
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading()
        );
        ManualDrive.driveOrHold(follower, powers);

        // relocalise button
        if (gamepad1.yWasPressed()) {
            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(0));
            // On the fly Pose creation, we dont recommend this for Autonomous. Only accepts radians for heading
            follower.setPose(cornerPose); // overrides our pose
        }

        follower.update();
        Pose robotPose = follower.pose(); // returns a Pose object
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));


        //----Turret Logic----
        yawAngle += gamepad2.right_stick_x * 0.01; // adds the current right stick value scaled down to the yaw angle
        yawAngle = Math.max(0.0, Math.min(1.0, yawAngle)); // clamps the value between 0.0 and 1.0
        yawServo.setPosition(yawAngle); // set the position of the servo to the final calculated angle

        pitchAngle += gamepad2.right_stick_y * 0.005;
        pitchAngle = Math.max(0.02, Math.min(0.35, pitchAngle));
        pitchServo.setPosition(pitchAngle);


        //----Shooter PID Control Logic----
        boolean aPressed = gamepad1.a || gamepad2.a;
        if (aPressed && !lastGamepadA) {
            shooterActive = !shooterActive;
        }
        lastGamepadA = aPressed;

        boolean isRunning = shooterActive || (gamepad1.right_trigger > 0.1) || (gamepad2.right_trigger > 0.1);

        double currentTime = getRuntime();
        double dt = Math.max(0.001, currentTime - lastTime);
        double currentTicksPerSec = shooter.getVelocity();

        double motorPower = 0.0;
        if (isRunning) {
            motorPower = ShooterConstants.calculatePower(currentTicksPerSec, dt, lastError);
            lastError = ShooterConstants.getTargetTicksPerSec() - currentTicksPerSec;
        } else {
            lastError = 0.0;
        }

        shooter.setPower(motorPower);
        lastTime = currentTime;

        // Telemetry
        telemetry.addData("Shooter Running", isRunning);
        telemetry.addData("Applied Power", String.format(Locale.US, "%.2f", motorPower));
        telemetry.addData("Target RPM", ShooterConstants.TARGET_RPM);
        telemetry.addData("Current RPM", String.format(Locale.US, "%.1f", ShooterConstants.ticksToRPM(currentTicksPerSec)));

        telemetry.addData("yawAngle", yawAngle);
        telemetry.addData("pitchAngle", pitchAngle);
    }

}





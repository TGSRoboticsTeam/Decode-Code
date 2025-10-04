package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "FlywheelTest", group = "Tools")

public class FlywheelTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Motor Setup
        DcMotor leftFlywheel = hardwareMap.get(DcMotor.class, "left_fly");
        DcMotor rightFlywheel = hardwareMap.get(DcMotor.class, "right_fly");

        // Sets the motor direction
        leftFlywheel.setDirection(DcMotor.Direction.FORWARD);
        rightFlywheel.setDirection(DcMotor.Direction.REVERSE);

        // Makes the motors stop moving when they receive an input of 0
        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo sweepServo = hardwareMap.get(Servo.class, "sweep_servo");

        while (!isStarted()) {

        }

        while (opModeIsActive()) {
            boolean sweep = gamepad1.a;
            boolean reset = gamepad1.b;

            if (sweep) {
                sweepServo.setPosition(1);
            } else if (reset) {
                sweepServo.setPosition(0);
            }

            leftFlywheel.setPower(1.0);
            rightFlywheel.setPower(1.0);
        }
    }
}
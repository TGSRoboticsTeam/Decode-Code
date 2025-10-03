package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

        while (!isStarted()) {

        }

        while (opModeIsActive()) {
            leftFlywheel.setPower(1.0);
            rightFlywheel.setPower(1.0);
        }
    }
}
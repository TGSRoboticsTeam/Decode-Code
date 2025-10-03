package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveClass {
    DcMotor leftFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightFrontDrive;
    DcMotor rightBackDrive;

    public DriveClass(HardwareMap hardwareMap) { // This is called a constructor, it's called when DriveClass is created.
        // Motor Setup
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Sets the motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Makes the motors stop moving when they receive an input of 0
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void fieldCentricDrive(double botHeading, double yaw, double pitch, double roll, double speedMultiplier) { // Yaw is rotation. Pitch is froward/backward. Roll is left/right.
        // Rotate the movement direction counter to the bots rotation
        double strafe = roll * Math.cos(-botHeading) - pitch * Math.sin(-botHeading);
        double drive = roll * Math.sin(-botHeading) + pitch * Math.cos(-botHeading);
        strafe = -strafe; // Here's the strafe inversion

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(yaw), 1);
        double frontLeftPower = (drive + strafe + yaw) / denominator;
        double backLeftPower = (drive - strafe + yaw) / denominator;
        double frontRightPower = (drive - strafe - yaw) / denominator;
        double backRightPower = (drive + strafe - yaw) / denominator;
        //telemetry.addData("fl, bl,fr,br: ", "%.2f %.2f %.2f %.2f",frontLeftPower,backLeftPower,frontRightPower,backRightPower);

        frontLeftPower *= speedMultiplier;
        frontRightPower *= speedMultiplier;
        backLeftPower *= speedMultiplier;
        backRightPower *= speedMultiplier;

        double roundDown = 0.05;
        if (Math.abs(frontLeftPower) <= roundDown) {
            frontLeftPower = 0;
        }
        if (Math.abs(frontRightPower) <= roundDown) {
            frontRightPower = 0;
        }
        if (Math.abs(backLeftPower) <= roundDown) {
            backLeftPower = 0;
        }
        if (Math.abs(backRightPower) <= roundDown) {
            backRightPower = 0;
        }

        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }

    public void robotCentricDrive(double yaw, double pitch, double roll, double speedMultiplier) { // Yaw is rotation. Pitch is froward/backward. Roll is left/right.
        fieldCentricDrive(0.0, yaw, pitch, roll, speedMultiplier);
    }


}
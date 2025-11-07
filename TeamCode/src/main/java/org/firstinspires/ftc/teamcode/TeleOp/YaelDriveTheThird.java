package org.firstinspires.ftc.teamcode.TeleOp;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//For FLL scrimmage
//@Disabled
@TeleOp(name = "YaelDriveTheThird", group = "TeleOp")

public class YaelDriveTheThird extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Motor Setup
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

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

        // FLYWHEEL SETUP
        // Motor Setup
        DcMotor leftFlywheel = hardwareMap.get(DcMotor.class, "left_fly");
        DcMotor rightFlywheel = hardwareMap.get(DcMotor.class, "right_fly");

        // Sets the motor direction
        leftFlywheel.setDirection(DcMotor.Direction.FORWARD);
        rightFlywheel.setDirection(DcMotor.Direction.REVERSE);

        // Makes the motors stop moving when they receive an input of 0
        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo leftFlap = hardwareMap.get(Servo.class, "left_flap");
        Servo rightFlap = hardwareMap.get(Servo.class, "right_flap");
        // Set up FtcDashboard telemetry
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //Telemetry dashboardTelemetry = dashboard.getTelemetry();

        double changeInSpeed = 0.35;
        boolean spinFlywheels = false;
        int flapTimer = 0;

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, // Change to left if doesn't work
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        while (!isStarted()) {

        }

        while (opModeIsActive()) {
            // Define joystick controls
            boolean startSpinning = gamepad1.left_trigger > 0.2;
            boolean stopSpinning = gamepad1.right_trigger > 0.2;

            boolean flipFlap = gamepad1.a;

            // Drive
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rot = gamepad1.right_stick_x;

            boolean slowDown = gamepad1.right_bumper;

            if (gamepad1.dpad_up) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bots rotation
            double strafe = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double drive = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            strafe = -strafe; // here's the strafe inversion

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rot), 1);
            double frontLeftPower = (drive + strafe + rot) / denominator;
            double backLeftPower = (drive - strafe + rot) / denominator;
            double frontRightPower = (drive - strafe - rot) / denominator;
            double backRightPower = (drive + strafe - rot) / denominator;
            //telemetry.addData("fl, bl,fr,br: ", "%.2f %.2f %.2f %.2f",frontLeftPower,backLeftPower,frontRightPower,backRightPower);

            if (slowDown) {
                frontLeftPower *= changeInSpeed;
                frontRightPower *= changeInSpeed;
                backLeftPower *= changeInSpeed;
                backRightPower *= changeInSpeed;
            }

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

            //////////////// OTHER COMPONENTS //////////////////
            // Spinners
            if (stopSpinning) {
                spinFlywheels = false;
            } else if (startSpinning) {
                spinFlywheels = true;
            }

            if (spinFlywheels) {
                leftFlywheel.setPower(1.0);
                rightFlywheel.setPower(1.0);
            } else {
                leftFlywheel.setPower(0.0);
                rightFlywheel.setPower(0.0);
            }

            // Flap
            if (flipFlap) {
                flapTimer = 1000;
            }

            if (flapTimer > 0) {
                flapTimer -= 1;
                leftFlap.setPosition(0.25);
                rightFlap.setPosition(0.75);
            } else {
                leftFlap.setPosition(0);
                rightFlap.setPosition(1.0);
            }

            ////////////// TELEMETRY //////////////
            /*telemetry.addData("Linear Slide", linearSlide.getCurrentPosition());

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Pitch (X)", "%.2f", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z)", "%.2f", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();*/
        }
    }
}
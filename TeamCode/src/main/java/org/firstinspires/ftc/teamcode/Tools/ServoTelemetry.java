package org.firstinspires.ftc.teamcode.Tools;

/*
ADD 'acmerobotics' INSIDE 'build.dependencies.gradle',
CHECK THE 'IntoTheDeep2' REPO FOR A BETTER IDEA OF WHAT TO DO,
ASK MR. CONNER FOR ADDITIONAL HELP
(SINCE LAST YEAR HE SAID HE HAD TO CHANGE SOME WEIRD NUMBERS ALSO TO GET IT TO WORK)
*/

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "ServoTelemetry", group = "Tools")

public class ServoTelemetry extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, "arm_servo");

        while (!isStarted()) {
            
        }

        // Keeps track of servo position...
        double servoPosition = servo.getPosition();

        // These are used to check if a button is being held down.
        boolean smallButtonEcho = false;
        boolean mediumButtonEcho = false;
        boolean largeButtonEcho = false;

        while (opModeIsActive()) {

            // Change these values to change the increment amount.
            double smallIncrementValue = 0.05;
            double mediumIncrementValue = smallIncrementValue * 5.0; // 0.25
            double largeIncrementValue = mediumIncrementValue * 5.0; // 1.25

            // Update inputs.
            boolean smallButtonPress = gamepad1.x;
            boolean mediumButtonPress = gamepad1.b;
            boolean largeButtonPress = gamepad1.a;

            // Update servo position variable.
            if (smallButtonPress && !smallButtonEcho) {
                servoPosition += smallIncrementValue;
            }
            if (mediumButtonPress && !mediumButtonEcho) {
                servoPosition += mediumIncrementValue;
            }
            if (largeButtonPress && !largeButtonEcho) {
                servoPosition += largeIncrementValue;
            }

            // Update echo.
            smallButtonEcho = smallButtonPress;
            mediumButtonEcho = mediumButtonPress;
            largeButtonEcho = largeButtonPress;

            servo.setPosition(servoPosition); // Sets the servo position...
            telemetry.addData("Servo Position: %.2f", servoPosition); // I think this'll print correctly...
            telemetry.update();
        }
    }
}

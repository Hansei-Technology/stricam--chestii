package htech.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "[UTIL] Double Servo Tester", group = "HTech")
public class DoubleServoTest extends LinearOpMode {
    public static String servoOne = "";
    public static String servoTwo = "";

    public static double servoOnePosition = 0.5;
    public static double servoTwoPosition = 0.5;

    public static double increment = 0.001;

    public static boolean simultaneousPositions = false;
    public static double sharedPosition = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo testedServoOne = hardwareMap.get(Servo.class, servoOne);
        Servo testedServoTwo = hardwareMap.get(Servo.class, servoTwo);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("[STATUS]", "DoubleServoTest Teleop is running.");

            if (gamepad1.dpad_up && servoOnePosition <= 1) {
                servoOnePosition += increment;
            } else if (gamepad1.dpad_down && servoOnePosition >= 0) {
                servoOnePosition -= increment;
            }

            if (gamepad1.y && servoTwoPosition <= 1) {
                servoTwoPosition += increment;
            } else if (gamepad1.a && servoTwoPosition >= 0) {
                servoTwoPosition -= increment;
            }

            if (simultaneousPositions) {
                servoOnePosition = sharedPosition;
                servoTwoPosition = sharedPosition;

                telemetry.addData("[INFO]", "Servo Shared Position: " + sharedPosition);
            }

            testedServoOne.setPosition(servoOnePosition);
            testedServoTwo.setPosition(servoTwoPosition);

            telemetry.addData("[INFO]", "Servo [1] Position: " + testedServoOne.getPosition());
            telemetry.addData("[INFO]", "Servo [2] Position: " + testedServoTwo.getPosition());
            telemetry.update();
        }
    }
}

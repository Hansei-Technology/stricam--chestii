package htech.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "[UTIL] Servo Tester", group = "HTech")
public class ServoTest extends LinearOpMode {
    public static String servoToTest = "";
    public static double position = 0.5;
    public static double increment = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo testedServo = hardwareMap.get(Servo.class, servoToTest);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("[STATUS]", "ServoTest Teleop is running.");

            if (gamepad1.dpad_up && position <= 1) {
                position += increment;
            } else if (gamepad1.dpad_down && position >= 0) {
                position -= increment;
            }

            testedServo.setPosition(position);

            telemetry.addData("[INFO]", "Servo Position: " + position);
            telemetry.update();
        }
    }
}

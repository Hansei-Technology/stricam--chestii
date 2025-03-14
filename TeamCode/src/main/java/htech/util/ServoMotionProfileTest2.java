package htech.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import htech.classes.DualServoController;

@Config
@TeleOp(name = "[UTIL] Double Servo Tester MOTION PROFILE", group = "HTech")
public class ServoMotionProfileTest2 extends LinearOpMode {
    Servo servo1, servo2;
    public static String servoOne = "";
    public static String servoTwo = "";
    public static double initialPoz = 0.5;
    public static double targetPoz = 0.5;
    public static double maxVel = 1.0;
    public static double accel = 1;
    public static double decel = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(Servo.class, servoOne);
        servo2 = hardwareMap.get(Servo.class, servoTwo);

        DualServoController dualServoController = new DualServoController(
                servo1, servo2,
                false, false, // ambele în direcție normală
                maxVel,  // viteză maximă
                accel,  // accelerație
                decel,  // decelerație
                initialPoz   // poziție inițială
        );

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a) {
                dualServoController.setTargetPosition(targetPoz);
            }

            dualServoController.update();
            dualServoController.telemetry(telemetry);
            telemetry.update();
        }
    }
}

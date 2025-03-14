package htech.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
@Config
public class CRServoTEster extends LinearOpMode {
    public static String name = "";
    public static double power = 0.5;
    CRServo servo;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(CRServo.class, name);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) servo.setPower(power);
            else if(gamepad1.b) servo.setPower(-power);
            else servo.setPower(0);
        }
    }
}

package htech.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import htech.subsystem.ExtendoSystem;
import htech.subsystem.LiftSystem;

@Config
@TeleOp(name = "[UTIL] ExtendoTester", group = "HTech")
public class ExtendoTester extends LinearOpMode {
    ExtendoSystem extendo;
    public static double kp = 0.05, ki = 0.0, kd = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        extendo = new ExtendoSystem(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.y) {
                extendo.goToMax();
            }
            if(gamepad1.b) {
                extendo.goToMid();
            }
            if(gamepad1.a) {
                extendo.goToGround();
            }

            //extendo.moveFree(gamepad1.right_trigger - gamepad1.left_trigger);

            extendo.pidController.p = kp;
            extendo.pidController.i = ki;
            extendo.pidController.d = kd;

            extendo.update();
            telemetry.addData("ExtendoPos", extendo.currentPos);
            telemetry.addData("[STATUS]", "LiftTester is running.");
            telemetry.update();
        }
    }
}

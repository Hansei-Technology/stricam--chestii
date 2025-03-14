package htech.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import htech.subsystem.LiftSystem;

@Config
@TeleOp(name = "[UTIL] LiftTester", group = "HTech")
public class LiftTester extends LinearOpMode {
    LiftSystem lift;
    public static double kp = 0.05, ki = 0.0, kd = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = new LiftSystem(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.y) {
                lift.goToHighBasket();
            }
            if(gamepad1.b) {
                lift.goToHighChamber();
            }
            if(gamepad1.a) {
                lift.goToGround();
            }

            lift.pidController.p = kp;
            lift.pidController.i = ki;
            lift.pidController.d = kd;

            lift.update();
            telemetry.addData("LiftPos", lift.currentPos);
            telemetry.addData("[STATUS]", "LiftTester is running.");
            telemetry.update();
        }
    }
}

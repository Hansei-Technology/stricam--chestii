package htech.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import htech.subsystem.ChassisMovement;

@TeleOp(name = "[UTIL] Chassis Tester", group = "HTech")
public class ChassisTester extends LinearOpMode {
    ChassisMovement chassis;
    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new ChassisMovement(hardwareMap);
        telemetry.addData("[STATUS]", "ChassisTester Teleop is running.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            chassis.updateMovementSlowRotation(gamepad1);
            telemetry.addData("[STATUS]", "ChassisTester Teleop is running.");
            telemetry.update();
        }
    }
}

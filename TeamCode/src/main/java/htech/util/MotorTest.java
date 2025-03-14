package htech.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import htech.subsystem.ChassisMovement;

@Config
@TeleOp(name = "[UTIL] Motor Tester", group = "HTech")
public class MotorTest extends LinearOpMode {
    public static String motorToTest = "";

    public static boolean isReversed= false;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx testedMotor = hardwareMap.get(DcMotorEx.class, motorToTest);
        if(isReversed)
            testedMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("[STATUS]", "MotorTest Teleop is running.");
            telemetry.addData("Current", testedMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("[INFO]", testedMotor.getCurrentPosition());

            testedMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            telemetry.update();
        }
    }
}

package htech.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "[UTIL] Double Motor Tester", group = "HTech")
public class DoubleMotorTest extends LinearOpMode {
    public static String testedMotorOne = "";
    public static String testedMotorTwo = "";

    public static boolean isReversedOne = false;
    public static boolean isReversedTwo = false;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motorOne = hardwareMap.get(DcMotorEx.class, testedMotorOne);
        if(isReversedOne)
            motorOne.setDirection(DcMotorEx.Direction.REVERSE);

        DcMotorEx motorTwo = hardwareMap.get(DcMotorEx.class, testedMotorTwo);
        if(isReversedTwo)
            motorTwo.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("[STATUS]", "MotorTest Teleop is running.");

            motorOne.setPower(gamepad1.right_trigger);
            motorTwo.setPower(gamepad1.right_trigger);

            telemetry.update();
        }
    }
}

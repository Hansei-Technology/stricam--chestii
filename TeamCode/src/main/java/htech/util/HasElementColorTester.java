package htech.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import htech.mechanism.intake.ColorSensor;
import htech.subsystem.IntakeSubsystem;

@TeleOp
@Config
public class HasElementColorTester extends LinearOpMode {
    //IntakeSubsystem intake;
    ColorSensor sensor;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = new ColorSensor(hardwareMap);
        //intake = new IntakeSubsystem(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Has Element: ", sensor.hasElement());
            telemetry.addData("Color: ", sensor.biggestColor);
            telemetry.addData("Max: ", sensor.max);
//            telemetry.addData("Has Element: ", intake.hasElement());
//            telemetry.addData("Color: ", intake.colorSensor.biggestColor);
//            telemetry.addData("Max: ", intake.colorSensor.max);
            telemetry.update();
        }
    }
}

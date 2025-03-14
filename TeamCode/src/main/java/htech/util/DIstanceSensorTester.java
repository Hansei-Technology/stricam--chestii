package htech.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Config
public class DIstanceSensorTester extends LinearOpMode {
    DistanceSensor sensor;
    public static String sensorName = "";
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(DistanceSensor.class, sensorName);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Distance", sensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

    }
}

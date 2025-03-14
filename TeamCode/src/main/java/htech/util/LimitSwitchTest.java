package htech.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Config
@TeleOp
public class LimitSwitchTest extends LinearOpMode {

    DigitalChannel limitSwitch;
    public static String sensorPort = "";

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limitSwitch = hardwareMap.get(DigitalChannel.class, sensorPort);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("output", limitSwitch.getState());
            telemetry.update();
        }

    }
}

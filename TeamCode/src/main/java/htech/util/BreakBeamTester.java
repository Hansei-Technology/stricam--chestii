package htech.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
@Config
public class BreakBeamTester extends LinearOpMode {
    DigitalChannel breakbeam;
    public static String name = "bb1";
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        breakbeam = hardwareMap.get(DigitalChannel.class, name);
        breakbeam.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Breakbeam", breakbeam.getState());
            telemetry.update();
        }
    }
}

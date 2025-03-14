package htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import htech.subsystem.ExtendoSystem;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
public class LimelightCacaCaca extends LinearOpMode {

    private Limelight3A limelight3A;
    private LLResult result;
    double pythonOutput[];
    Follower follower;
    ExtendoSystem extendo;



    @Override
    public void runOpMode() throws InterruptedException {
        extendo = new ExtendoSystem(hardwareMap);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        extendo.pidEnabled = false;

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(180)));

        limelight3A.pipelineSwitch(5);
        limelight3A.start();

        waitForStart();

        while(opModeIsActive()) {
            result = limelight3A.getLatestResult();
            pythonOutput = result.getPythonOutput();

            telemetry.addData("Has sample", pythonOutput[0]);
            telemetry.addData("cameraX", pythonOutput[1]);
            telemetry.addData("cameraY", pythonOutput[2]);
            telemetry.addData("extendo", extendo.currentPos);
            telemetry.addData("robotX", follower.getPose().getX());
            telemetry.addData("robotY", follower.getPose().getY());
            telemetry.update();
            follower.update();
            extendo.update();
        }
    }
}
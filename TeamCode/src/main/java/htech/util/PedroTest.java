package htech.util;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;
import static com.pedropathing.follower.FollowerConstants.leftFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorDirection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;

import java.util.Arrays;
import java.util.List;

import pedroPathing.constants.*;

@Config
@TeleOp
public class PedroTest extends OpMode {
    private DashboardPoseTracker dashboardPoseTracker;
    private Telemetry telemetryA;
    private Follower follower;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;


    public static double START_X = 55, START_Y = 66, START_HEADING = Math.toRadians(90);
    public static double END_X = 45, END_Y = 76, END_HEADING = Math.toRadians(90);

    /**
     * This initializes the PoseUpdater, the mecanum drive motors, and the FTC Dashboard telemetry.
     */
    @Override
    public void init() {


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        dashboardPoseTracker = new DashboardPoseTracker(follower.poseUpdater);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        leftFront.setDirection(leftFrontMotorDirection);
        leftRear.setDirection(leftRearMotorDirection);
        rightFront.setDirection(rightFrontMotorDirection);
        rightRear.setDirection(rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Drawing.drawRobot(follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

        follower.resetOffset();
        follower.setStartingPose(new Pose(START_X, START_Y, START_HEADING));


        PathChain line = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(START_X, START_Y),
                                new Point(END_X, END_Y)
                        )
                )
                .setLinearHeadingInterpolation(START_HEADING, END_HEADING)
                .build();
        follower.followPath(line);
    }
    @Override
    public void loop() {
        follower.update();
        dashboardPoseTracker.update();



        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", follower.getPose().getHeading());
        telemetryA.addData("total heading", follower.getTotalHeading());
        telemetryA.update();

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(follower.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
}

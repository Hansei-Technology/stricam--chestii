package htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.pathgen.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import htech.config.PositionsExtendo;
import htech.mechanism.LimeLightWrapper;
import htech.subsystem.ExtendoSystem;
import htech.subsystem.IntakeSubsystem;
import htech.subsystem.LiftSystem;
import htech.subsystem.OuttakeSubsystem;
import htech.subsystem.RobotSystems;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
@Config
public class LimeLightPinkTest extends LinearOpMode {
    public static double extendoMultiplyer = 0.4;
    public static double freeTerm = 0;
    boolean isretracting = false;
    boolean isDown = false;
    ElapsedTime timer;

    public enum SubmersibleState {
        IDLE,
        EXTENDING,
        WAITING,
        TRANSFERING,
        COLLECTING
    }
    SubmersibleState subCS = SubmersibleState.IDLE;

    private LimeLightWrapper limelight;
    private IntakeSubsystem intake;
    private ExtendoSystem extendo;
    private RobotSystems robot;
    private LiftSystem lift;
    private OuttakeSubsystem outtake;
    private Follower follower;
    private Path path;
    private int loopCount=0;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight = new LimeLightWrapper(hardwareMap);
        timer = new ElapsedTime();
        intake = new IntakeSubsystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);
        lift = new LiftSystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
        robot = new RobotSystems(extendo, lift, intake, outtake);


        limelight.start();

        intake.goToLimeLight();

        while(opModeInInit()) {
            limelight.update();

            telemetry.addData("pythonOutput0", limelight.pythonOutput[0]);
            telemetry.addData("pythonOutput1", limelight.pythonOutput[1]);
            telemetry.addData("pythonOutput2", limelight.pythonOutput[2]);
            telemetry.addData("pythonOutput3", limelight.pythonOutput[3]);
            telemetry.update();
        }



        while (opModeIsActive()) {
            if(subCS == SubmersibleState.IDLE) {
                if (limelight.valid && gamepad1.a) {
                    follower.setPose(new Pose(0, 0, Math.toRadians(0)));
                    path = new Path(
                            new BezierLine(
                                    new Point(0, 0, Point.CARTESIAN),
                                    new Point(0, -limelight.X * 0.3937, Point.CARTESIAN)
                            )
                    );
                    path.setConstantHeadingInterpolation(Math.toRadians(0));

                    follower.followPath(path, true);
                    intake.goDown();
                    extendo.goToPos((int)limelight.Y);
                    if(Math.abs(limelight.HEADING - 90) < 15) intake.rotation.rotateToAngle(90);
                    else intake.rotation.goToFlipped();
                            //intake.rotation.rotateToAngle(-(int) limelight.HEADING);
                    subCS = SubmersibleState.EXTENDING;
                } else {
                    limelight.update();
                }
            } else if(subCS == SubmersibleState.EXTENDING) {
                if(follower.getCurrentTValue() > 0.99 && extendo.isAtPosition()) {
                    timer.reset();
                    subCS = SubmersibleState.WAITING;
                }
            } else if(subCS == SubmersibleState.WAITING) {
                if(timer.milliseconds() > 200) {
                    subCS = SubmersibleState.COLLECTING;
                    intake.collect(); //fast collect
                }

            } else {
                if(gamepad1.b) {
                    subCS = SubmersibleState.IDLE;
                    extendo.goToGround();
                    outtake.claw.open();
                    intake.goToLimeLight();
                }
            }
            telemetry.addData("pythonOutput0", limelight.pythonOutput[0]);
            telemetry.addData("pythonOutput1", limelight.pythonOutput[1]);
            telemetry.addData("pythonOutput2", limelight.pythonOutput[2]);
            telemetry.addData("pythonOutput3", limelight.pythonOutput[3]);
            telemetry.addData("extendopos", extendo.currentPos);
            telemetry.addData("subCS", subCS);
            telemetry.addData("X", limelight.X);
            telemetry.addData("Y", limelight.Y);
            telemetry.addData("HEADING", limelight.HEADING);
            telemetry.update();
            follower.update();
            robot.update();
        }
    }
}

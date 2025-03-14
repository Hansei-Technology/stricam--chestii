package htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import htech.subsystem.ExtendoSystem;
import htech.subsystem.IntakeSubsystem;
import htech.subsystem.LiftSystem;
import htech.subsystem.OuttakeSubsystem;
import htech.subsystem.RobotSystems;

@TeleOp(name = "Limelight Test", group = "Tests")
@Config
public class LimelightTest extends LinearOpMode {
    public static int extendoLime = 30;
    public static int timeGlis = 550;
    boolean isretracting = false;
    boolean isDown = false;
    ElapsedTime timer;

    public enum SubmersibleState {
        IDLE,
        EXTENDING,
        TRANSFERING,
        COLLECTING
    }
    SubmersibleState subCS = SubmersibleState.IDLE;

    private Limelight3A limelight;
    private IntakeSubsystem intake;
    private ExtendoSystem extendo;
    private RobotSystems robot;
    private LiftSystem lift;
    private OuttakeSubsystem outtake;
    public static double a=0.000223, b=0.281, c=89.11;
    private int loopCount=0;

    public int getExtendoPos(double x){
        return (int) ((int) a*Math.pow(x, 2) + b*x + c);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        timer = new ElapsedTime();
        intake = new IntakeSubsystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);
        lift = new LiftSystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        robot = new RobotSystems(extendo, lift, intake, outtake);

        limelight.pipelineSwitch(0);
        limelight.start();

        intake.goDown();
        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a) {
                subCS = SubmersibleState.EXTENDING;
                intake.goDown();
                intake.rotation.goToFlipped();
            }
            if(gamepad1.b) {
                subCS = SubmersibleState.IDLE;
                extendo.goToGround();
            }

            switch (subCS) {
                case IDLE:
                    if(gamepad1.a) {
                        subCS = SubmersibleState.EXTENDING;
                        intake.goDown();
                    }
                    break;
                case EXTENDING:
                    extendo.moveFree(0.3);
                    LLResult result = limelight.getLatestResult();
                    double[] pythonOutput = result.getPythonOutput();
                    int validContours = (int) pythonOutput[0];  // 1 dacă există contururi, 0 altfel
//                    double heading = Math.atan(Math.tan(Math.toRadians(pythonOutput[5])));// Unghiul conturului
                    double heading= pythonOutput[5];
                    if(validContours == 1 && loopCount==10) {
                        intake.rotation.rotateToAngle(90-(int) heading);
                        extendo.goToPos(extendo.currentPos + extendoLime);
                        subCS = SubmersibleState.COLLECTING;
                        timer.reset();
                    }
                    break;
                case COLLECTING:
                    result = limelight.getLatestResult();
                    pythonOutput = result.getPythonOutput();
                    validContours = (int) pythonOutput[0];  // 1 dacă există contururi, 0 altfel
//                    heading = Math.atan(Math.tan(Math.toRadians(pythonOutput[1])));// Unghiul conturului
                    heading = pythonOutput[5];
                    telemetry.addData("Heading", pythonOutput[5]);


                    if(validContours == 1 && !extendo.isAtPosition()) {
                        intake.rotation.rotateToAngle(90-(int) heading);
                    }

                    if(extendo.isAtPosition() && timer.milliseconds() > timeGlis) {
                        intake.collect(); //fast collect
                        subCS = SubmersibleState.TRANSFERING;

                    }
                    break;
                case TRANSFERING:

                    break;
            }

            robot.update();
            telemetry.addData("State", subCS);
            telemetry.update();
            if(loopCount==10){
                loopCount=0;
            } else {
                loopCount++;
            }
        }
    }
}

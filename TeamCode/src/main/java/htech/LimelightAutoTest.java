package htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import htech.config.PositionsIntake;
import htech.subsystem.ExtendoSystem;
import htech.subsystem.IntakeSubsystem;
import htech.subsystem.LiftSystem;
import htech.subsystem.OuttakeSubsystem;
import htech.subsystem.RobotSystems;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@TeleOp
public class LimelightAutoTest extends LinearOpMode {

    //Mechanisms
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    RobotSystems robotSystems;
    ElapsedTime timer;
    ElapsedTime matchTimer;
    Limelight3A ll;
    double llpython[];
    LLResult llResult;


    //Pedro
    Follower follower;
    Path preload;
    PathChain collectSamples, failSafe1, failSafe;
    Path score1, score2, score3, score4, score5;
    PathChain wall;
    Path park;
    Path collectSampleBasket, goToBasket;
    Path dropSampleFromSub;
    PathChain collectSamples2;
    Path goToSampleLL;
    Path pula;


    //States
    public enum STATES{
        IDLE,
        SPECIMEN, SCORING_SPECIMEN,
        COLLECTING_SAMPLES, COLLECTING_SPECIMEN,
        SCORE,
        WALL,
        PARK,
        PARKED,
        MOVING, WAITING, TRANSFER,
        CHECK_SPECIMEN, FAIL_SAFE,
        LL_COLLECTING, LL_MOVING, LL_GOING_DOWN, LL_CHECK,
        CHECKPOINT, WALL1,
        COLLECTING_SAMPLE,
        COLLECTING_SAMPLE1,
        COLLECTING_SAMPLE2, COLLECTING_SAMPLE3,
        SCORE_BASKET, SCORE_BASKET2, SCORE_BASKET3,
        DROPPING_SAMPLE, DROPPING_SAMPLE2, PULA, MUIE
    }
    public enum SCORING_STATES{
        IDLE,
        SCORE1,
        SCORE2,
        SCORE3,
        SCORE4,
        SCORE5
    }
    STATES CS = STATES.IDLE, NS = STATES.IDLE;
    SCORING_STATES SCORING_CS = SCORING_STATES.IDLE;


    //Coordinates
    public static double startX = 0, startY = 0, startH = 180;
    public static double preloadX = -30.5, preloadY = -5.5, preloadH = startH;

    public static double safe1Sample1X = 0, safe1Sample1Y = 32;
    public static double safe2Sample1X = -30, safe2Sample1Y = 15;
    public static double safe3Sample1X = -60, safe3Sample1Y = 37;
    public static double sample1X = -48, sample1Y = 37, sample1H = 180;
    public static double human1X = -23, human1Y = 37, human1H = 180;

    public static double safeSample2X = -48, safeSample2Y = 30;
    public static double sample2X = -48, sample2Y = 45, sample2H = 180;
    public static double human2X = -24.5, human2Y = 45, human2H = 180;

    public static double safeSample3X = -32.5, safeSample3Y = 37;
    public static double sample3X = -48, sample3Y = 52, sample3H = 180;
    public static double human3X = -3.5, human3Y = 51.5;

    public static double checkpointX = -23, checkpointY = 21, checkpointH = 180;

    public static double score1X = -31.5, score1Y = -2, scoreH = 180;
    public static double score2X = -31.5, score2Y = -4;
    public static double score3X = -31.5, score3Y = -7;
    public static double score4X = -31.5, score4Y = -8;
    public static double safeScoreX = -14, safeScoreY = -7;

    public static double score5X = -31.5, score5Y = -10;
    public static double sampleDropX = -23, sampleDropY = 0, sampleDropH = 50;

    public static double specimenX = -3.5, specimenY = 27.5, specimenH = 180;
    public static double safe1SpecimenX = -15, safe1SpecimenY = -5;
    public static double safe2SpecimenX = -15, safe2SpecimenY = 30;

    public static double parkX = -3, parkY = 30, parkH = 60;

    public static double sampleBasketX = -24, sampleBasketY = 20, sampleBasketH = 35;
    public static double basketX = -5, basketY = -56, basketH = 115;

    public static double safeDropX = -24, safeDropY = -30;

    public static int pipelineNumber = 5;

    public static double safePulaX = 0, safePulaY = -40;

    //Booleans
    boolean basket = false;
    boolean parking = false;
    boolean firstTime = true;
    boolean collectedFromSub = false;
    boolean sub = false;



    //Timers
    public static double timeToCollect = 100;
    public static double matchTime = 30;
    public static double timeToGoDownSample = 200;
    public static double timeToCollectSample = 200;
    public double timeToWait = 0;
    public static double timeToDropSample = 100;
    public static double timeToCheck = 100;




    //Speed
    public static double speed = 1;

    public static double pulaX = -10, pulaY = 4, pulaH = 180;

    //Claw rotations
    public static double normal = PositionsIntake.normalRotation;
    boolean diagonalRight = false;
    boolean diagonalLeft = false;
    public static double perpendicular = PositionsIntake.perpendicularRotation;
    public static double rotPos = 0;
    public static String rotationString = "";
    public static String normalString = "Normal";
    public static String diagonalRightString = "Diagonal Right";
    public static String diagonalLeftString = "Diagonal Left";
    public static String perpendicularString = "Perpendicular";

    public static double offsetll = -4.4;

    public static int extendoPos = 0;
    public static double preloadOffset = 0;

    boolean isReading = false;
    boolean retractIntake = false;

    double rotDegrees = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        lift = new LiftSystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);
        robotSystems = new RobotSystems(extendo, lift, intakeSubsystem, outtakeSubsystem);
        timer = new ElapsedTime();
        matchTimer = new ElapsedTime();
        ll = hardwareMap.get(Limelight3A.class, "limelight");

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(startX, startY, Math.toRadians(startH)));

        intakeSubsystem.goToWall();
        outtakeSubsystem.init();
        outtakeSubsystem.claw.close();
        extendo.pidEnabled = true;

        pula = new Path(
                new BezierLine(
                        new Point(preloadX, preloadY, Point.CARTESIAN),
                        new Point(pulaX, pulaY, Point.CARTESIAN)
                )
        );
        pula.setConstantHeadingInterpolation(Math.toRadians(pulaH));

        preload = new Path(
                new BezierLine(
                        new Point(startX, startY, Point.CARTESIAN),
                        new Point(preloadX, preloadY, Point.CARTESIAN)
                )
        );
        preload.setConstantHeadingInterpolation(Math.toRadians(preloadH));

        collectSamples = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pulaX, pulaY, Point.CARTESIAN),
                                new Point(safe1Sample1X, safe1Sample1Y, Point.CARTESIAN),
                                new Point(safe2Sample1X, safe2Sample1Y, Point.CARTESIAN),
                                new Point(safe3Sample1X, safe3Sample1Y, Point.CARTESIAN),
                                new Point(sample1X, sample1Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(sample1H))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierLine(
                                new Point(sample1X, sample1Y, Point.CARTESIAN),
                                new Point(human1X, human1Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(human1H))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierCurve(
                                new Point(human1X, human1Y, Point.CARTESIAN),
                                new Point(safeSample2X, safeSample2Y, Point.CARTESIAN),
                                new Point(sample2X, sample2Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(sample2H))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierLine(
                                new Point(sample2X, sample2Y, Point.CARTESIAN),
                                new Point(human2X, human2Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(human2H))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierCurve(
                                new Point(human2X, human2Y, Point.CARTESIAN),
                                new Point(safeSample3X, safeSample3Y, Point.CARTESIAN),
                                new Point(sample3X, sample3Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(sample3H))
                .addPath(
                        new BezierLine(
                                new Point(sample3X, sample3Y, Point.CARTESIAN),
                                new Point(human3X, human3Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(sample3H))
                .build();

        collectSamples2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(sampleDropX, sampleDropY, Point.CARTESIAN),
                                new Point(safePulaX, safePulaY, Point.CARTESIAN),
                                new Point(safe3Sample1X, safe3Sample1Y, Point.CARTESIAN),
                                new Point(sample1X, sample1Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(sampleDropH), Math.toRadians(sample1H))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierLine(
                                new Point(sample1X, sample1Y, Point.CARTESIAN),
                                new Point(human1X, human1Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(human1H))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierCurve(
                                new Point(human1X, human1Y, Point.CARTESIAN),
                                new Point(safeSample2X, safeSample2Y, Point.CARTESIAN),
                                new Point(sample2X, sample2Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(sample2H))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierLine(
                                new Point(sample2X, sample2Y, Point.CARTESIAN),
                                new Point(human2X, human2Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(human2H))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierCurve(
                                new Point(human2X, human2Y, Point.CARTESIAN),
                                new Point(safeSample3X, safeSample3Y, Point.CARTESIAN),
                                new Point(sample3X, sample3Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(sample3H))
                .addPath(
                        new BezierLine(
                                new Point(sample3X, sample3Y, Point.CARTESIAN),
                                new Point(human3X, human3Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(sample3H))
                .build();


        wall = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(preloadX, preloadY, Point.CARTESIAN),
                                new Point(checkpointX, checkpointY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Point(checkpointX, checkpointY, Point.CARTESIAN),
                                new Point(specimenX, specimenY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();



        score1 = new Path(
                new BezierCurve(
                        new Point(human3X, human3Y, Point.CARTESIAN),
                        new Point(safeScoreX, safeScoreY, Point.CARTESIAN),
                        new Point(score1X, score1Y, Point.CARTESIAN)
                )
        );
        score1.setConstantHeadingInterpolation(Math.toRadians(scoreH));

        score2 = new Path(
                new BezierCurve(
                        new Point(specimenX, specimenY, Point.CARTESIAN),
                        new Point(safeScoreX, safeScoreY, Point.CARTESIAN),
                        new Point(score2X, score2Y, Point.CARTESIAN)
                )
        );
        score2.setConstantHeadingInterpolation(Math.toRadians(scoreH));

        score3 = new Path(
                new BezierCurve(
                        new Point(specimenX, specimenY, Point.CARTESIAN),
                        new Point(safeScoreX, safeScoreY, Point.CARTESIAN),
                        new Point(score3X, score3Y, Point.CARTESIAN)
                )
        );
        score3.setConstantHeadingInterpolation(Math.toRadians(scoreH));

        score4 = new Path(
                new BezierCurve(
                        new Point(specimenX, specimenY, Point.CARTESIAN),
                        new Point(safeScoreX, safeScoreY, Point.CARTESIAN),
                        new Point(score4X, score4Y, Point.CARTESIAN)
                )
        );
        score4.setConstantHeadingInterpolation(Math.toRadians(scoreH));

        score5 = new Path(
                new BezierCurve(
                        new Point(specimenX, specimenY, Point.CARTESIAN),
                        new Point(safeScoreX, safeScoreY, Point.CARTESIAN),
                        new Point(score5X, score5Y, Point.CARTESIAN)
                )
        );
        score5.setConstantHeadingInterpolation(Math.toRadians(scoreH));

        failSafe1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(human3X, human3Y, Point.CARTESIAN),
                                new Point(human3X - 5, human3Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(preloadH)
                .setPathEndTimeoutConstraint(200)
                .addPath(
                        new BezierLine(
                                new Point(human3X - 5, human3Y, Point.CARTESIAN),
                                new Point(human3X + 0.5, human3Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(preloadH)
                .setPathEndTimeoutConstraint(0)
                .build();

        failSafe = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(specimenX, specimenY, Point.CARTESIAN),
                                new Point(specimenX - 5, specimenY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(preloadH)
                .setPathEndTimeoutConstraint(300)
                .addPath(
                        new BezierLine(
                                new Point(specimenX - 5, specimenY, Point.CARTESIAN),
                                new Point(specimenX + 0.5, specimenY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(preloadH)
                .setPathEndTimeoutConstraint(0)
                .build();

        park = new Path(
                new BezierCurve(
                        new Point(basketX, basketY, Point.CARTESIAN),
                        new Point(safe1SpecimenX, safe1SpecimenY, Point.CARTESIAN),
                        new Point(parkX, parkY, Point.CARTESIAN)
                )
        );
        park.setLinearHeadingInterpolation(Math.toRadians(basketH), Math.toRadians(parkH));



        collectSampleBasket = new Path(
                new BezierCurve(
                        new Point(preloadX, preloadY, Point.CARTESIAN),
                        new Point(safe1SpecimenX, safe1SpecimenY, Point.CARTESIAN),
                        new Point(sampleBasketX, sampleBasketY, Point.CARTESIAN)
                )
        );
        collectSampleBasket.setLinearHeadingInterpolation(Math.toRadians(scoreH), Math.toRadians(sampleBasketH));

        goToBasket = new Path(
                new BezierLine(
                        new Point(sampleBasketX, sampleBasketY, Point.CARTESIAN),
                        new Point(basketX, basketY, Point.CARTESIAN)
                )
        );
        goToBasket.setLinearHeadingInterpolation(Math.toRadians(sampleBasketH), Math.toRadians(basketH));

        dropSampleFromSub = new Path(
                new BezierCurve(
                        new Point(pulaX, pulaY, Point.CARTESIAN),
                        new Point(safeDropX, safeDropY, Point.CARTESIAN),
                        new Point(sampleDropX, sampleDropY, Point.CARTESIAN)
                )
        );
        dropSampleFromSub.setLinearHeadingInterpolation(Math.toRadians(preloadH), Math.toRadians(sampleDropH));

        follower.setMaxPower(0.5);

        while(opModeInInit()) {
            rotDegrees += (gamepad1.right_trigger - gamepad1.left_trigger) * 0.01;
            if(rotDegrees > 90) rotDegrees = 90;
            if(rotDegrees < -90) rotDegrees = -90;

            ll.start();
            ll.pipelineSwitch(pipelineNumber);

            telemetry.addData("rotDegrees", rotDegrees);
            telemetry.update();
        }

        matchTimer.reset();

        while(opModeIsActive()){

            double llMultiX = 0.4;
            double llMultiY = 2.4;
            double llNormalY = 223;

            switch (CS){

                case IDLE:
                    llResult = ll.getLatestResult();
                    llpython = llResult.getPythonOutput();
                    CS = STATES.LL_MOVING;
                    break;

                case MOVING:
                    if(!follower.isBusy()){
                        CS = NS;
                        timer.reset();
                    }
                    break;

                case LL_MOVING:
                    goToSampleLL = new Path(
                            new BezierLine(
                                    new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                                    new Point(follower.getPose().getX(), follower.getPose().getY() + (-llpython[1] * llMultiX) + offsetll )
                            )
                    );
                    goToSampleLL.setConstantHeadingInterpolation(Math.toRadians(preloadH));
                    goToSampleLL.setPathEndTimeoutConstraint(300);
                    if(llpython[0] == 1){
//                        follower.followPath(goToSampleLL, true);
                        extendo.goToPos((int) (llNormalY + llpython[2] * llMultiY));
                        intakeSubsystem.goDown();
                        intakeSubsystem.rotation.rotateToAngle((int)rotDegrees);
                        CS = STATES.MOVING;
                        NS = STATES.LL_COLLECTING;
                    }
                    break;

                case LL_COLLECTING:
                    if(!follower.isBusy() && extendo.isAtPosition() && timer.milliseconds() > 300){
                        intakeSubsystem.bar.goToCollect();
                        timeToWait = timeToCollectSample;
                        CS = STATES.WAITING;
                        NS = STATES.LL_GOING_DOWN;
                        timer.reset();
                    }
                    break;

                case LL_GOING_DOWN:
                    intakeSubsystem.claw.close();
                    CS = STATES.WAITING;
                    NS = STATES.LL_CHECK;
                    timeToWait = timeToCollect;
                    timer.reset();
                    break;

                case LL_CHECK:
                    intakeSubsystem.goToWall();
                    if(timer.milliseconds() > 200){
                        intakeSubsystem.claw.open();
                        CS = STATES.LL_MOVING;
                    }
                    break;


            }

            follower.update();
            robotSystems.update();
            telemetry.addData("STATE", CS);
            telemetry.addData("TIMER", timer.milliseconds());
            telemetry.addData("MATCH TIMER", matchTimer.seconds());
            telemetry.addData("has sample", llpython[0]);
            telemetry.addData("x", llpython[1]);
            telemetry.addData("y", llpython[2]);
            telemetry.update();

        }

    }
}

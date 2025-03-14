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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import htech.subsystem.ExtendoSystem;
import htech.subsystem.IntakeSubsystem;
import htech.subsystem.LiftSystem;
import htech.subsystem.OuttakeSubsystem;
import htech.subsystem.RobotSystems;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "[AUTO] 5 + 0", group = "HTECH")
public class AutoSpecimene extends LinearOpMode {

    //Mechanisms
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    RobotSystems robotSystems;
    ElapsedTime timer;
    ElapsedTime matchTimer;


    //Pedro
    Follower follower;
    Path preload;
    PathChain collectSamples, failSafe1, failSafe;
    Path score1, score2, score3, score4;
    Path wall;
    Path park;
    Path checkpoint, wall1;


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
        CHECKPOINT, WALL1
    }
    public enum SCORING_STATES{
        IDLE,
        SCORE1,
        SCORE2,
        SCORE3,
        SCORE4
    }
    STATES CS = STATES.IDLE, NS = STATES.IDLE;
    SCORING_STATES SCORING_CS = SCORING_STATES.IDLE;


    //Coordinates
    public static double startX = 0, startY = 0, startH = 180;
    public static double preloadX = -30.5, preloadY = -5.5, preloadH = startH;

    public static double safe1Sample1X = -5, safe1Sample1Y = 32;
    public static double safe2Sample1X = -30, safe2Sample1Y = 15;
    public static double safe3Sample1X = -60, safe3Sample1Y = 37;
    public static double sample1X = -48, sample1Y = 37, sample1H = 180;
    public static double human1X = -23, human1Y = 37, human1H = 180;

    public static double safeSample2X = -48, safeSample2Y = 30;
    public static double sample2X = -48, sample2Y = 45, sample2H = 180;
    public static double human2X = -24.5, human2Y = 45, human2H = 180;

    public static double safeSample3X = -32.5, safeSample3Y = 37;
    public static double sample3X = -48, sample3Y = 52, sample3H = 180;
    public static double human3X = -19, human3Y = 52;

    public static double checkpointX = -23, checkpointY = 30, checkpointH = 180;

    public static double score1X = -31.5, score1Y = -2, scoreH = 180;
    public static double score2X = -31.5, score2Y = -4;
    public static double score3X = -31.5, score3Y = -7;
    public static double score4X = -31.5, score4Y = -8;
    public static double safeScoreX = -14, safeScoreY = -7;

    public static double specimenX = -3, specimenY = 30, specimenH = 180;
    public static double safe1SpecimenX = -15, safe1SpecimenY = -5;
    public static double safe2SpecimenX = -15, safe2SpecimenY = 30;

    public static double parkX = -10, parkY = 30, parkH = 65;


    boolean firstTime = true;
    //Timers
    public static double timeToCollect = 100;
    public static double matchTime = 30;
    public static double timeToScore = 200;
    public double timeToWait = 0;


    public static double speed = 1;



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

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(startX, startY, Math.toRadians(startH)));

        intakeSubsystem.goToWall();
        outtakeSubsystem.init();
        outtakeSubsystem.claw.close();
        extendo.pidEnabled = true;

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
                                new Point(preloadX, preloadY, Point.CARTESIAN),
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

        checkpoint = new Path(
                new BezierLine(
                        new Point(human3X, human3Y, Point.CARTESIAN),
                        new Point(checkpointX, checkpointY, Point.CARTESIAN)
                )
        );
        checkpoint.setConstantHeadingInterpolation(Math.toRadians(checkpointH));

        wall1 = new Path(
                new BezierLine(
                        new Point(checkpointX, checkpointY, Point.CARTESIAN),
                        new Point(specimenX, specimenY, Point.CARTESIAN)
                )
        );
        wall1.setConstantHeadingInterpolation(Math.toRadians(180));

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

        wall = new Path(
                new BezierCurve(
                        new Point(preloadX, preloadY, Point.CARTESIAN),
                        new Point(safe1SpecimenX, safe1SpecimenY, Point.CARTESIAN),
                        new Point(safe2SpecimenX, safe2SpecimenY, Point.CARTESIAN),
                        new Point(specimenX, specimenY, Point.CARTESIAN)
                )
        );
        wall.setConstantHeadingInterpolation(Math.toRadians(specimenH));
        wall.setPathEndTimeoutConstraint(255);

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
                        new Point(preloadX, preloadY, Point.CARTESIAN),
                        new Point(safe1SpecimenX, safe1SpecimenY, Point.CARTESIAN),
                        new Point(parkX, parkY, Point.CARTESIAN)
                )
        );
        park.setLinearHeadingInterpolation(Math.toRadians(scoreH), Math.toRadians(parkH));

        follower.setMaxPower(speed);
        follower.followPath(preload);

        waitForStart();

        matchTimer.reset();

        while(opModeIsActive() && matchTimer.seconds() < matchTime){

            switch (CS){

                case IDLE:
                    CS = STATES.SPECIMEN;
                    break;

                case SPECIMEN:
                    if(robotSystems.transferState == RobotSystems.TransferStates.IDLE || robotSystems.transferState == RobotSystems.TransferStates.GOING_TO_AFTER_TRANSFER) {

                        lift.goToHighChamber();
                        outtakeSubsystem.goToSpecimenPrescore();
                        outtakeSubsystem.funny.extend();

                        CS = STATES.MOVING;
                        NS = STATES.SCORING_SPECIMEN;
                        firstTime = true;
                    }
                    break;

                case SCORING_SPECIMEN:
                    if(firstTime) {
                        robotSystems.scoreSpecimen();
                        firstTime = false;
                    }

                    timer.reset();
                    if(robotSystems.scoreSpecimenState == RobotSystems.scoreSpecimenStates.IDLE){
                        switch (SCORING_CS){
                            case IDLE:
                                CS = STATES.COLLECTING_SAMPLES;
                                break;
                            case SCORE1:
                                CS = STATES.WALL;
                                break;
                            case SCORE2:
                                CS = STATES.WALL;
                                break;
                            case SCORE3:
                                CS = STATES.WALL;
                                break;
                            case SCORE4:
                                CS = STATES.PARK;
                                break;
                        }
                    }
                    break;

                case MOVING:
                    if(!follower.isBusy()){
                        CS = NS;
                        timer.reset();
                        firstTime = true;

                    }
                    break;

                case WAITING:
                    if(timer.milliseconds() > timeToWait){
                        CS = NS;
                        timer.reset();
                        firstTime = true;
                    }
                    break;

                case COLLECTING_SAMPLES:
                    timer.reset();
                    follower.followPath(collectSamples);
                    lift.goToGround();
                    robotSystems.collectSpecimen();
                    intakeSubsystem.goToWall();
                    CS = STATES.MOVING;
                    NS = STATES.CHECKPOINT;
                    SCORING_CS = SCORING_STATES.SCORE1;
                    break;

                case CHECKPOINT:
                    follower.followPath(checkpoint);
                    CS = STATES.MOVING;
                    NS = STATES.WALL1;
                    break;

                case WALL1:
                    follower.followPath(wall1);
                    CS = STATES.MOVING;
                    NS = STATES.COLLECTING_SPECIMEN;
                    break;

                case COLLECTING_SPECIMEN:
                    outtakeSubsystem.claw.close();
                    timer.reset();
                    timeToWait = timeToCollect;
                    CS = STATES.WAITING;
                    NS = STATES.SCORE;
                    break;

//                case CHECK_SPECIMEN:
//                    if(timer.milliseconds() > timeToCheck) {
//                        if(intakeSubsystem.hasElement()){
//                            CS = STATES.TRANSFER;
//                            robotSystems.transfer();
//                        }
//                        else{
//                            intakeSubsystem.claw.open();
//                            timer.reset();
//                            timeToWait = 100;
//                            CS = STATES.WAITING;
//                            NS = STATES.FAIL_SAFE;
//                        }
//                    }
//
//                    break;
//
//                case FAIL_SAFE:
//                    if(SCORING_CS == SCORING_STATES.SCORE1){
//                        follower.followPath(failSafe1);
//                        CS = STATES.MOVING;
//                        NS = STATES.COLLECTING_SPECIMEN;
//                    }
//                    else{
//                        follower.followPath(failSafe);
//                        CS = STATES.MOVING;
//                        NS = STATES.COLLECTING_SPECIMEN;
//                    }
//                    break;

                case SCORE:
                    lift.goToHighBasket();
                    outtakeSubsystem.goToSpecimenPrescore();
                    switch (SCORING_CS){
                        case SCORE1:
                            follower.followPath(score1);
                            break;
                        case SCORE2:
                            follower.followPath(score2);
                            break;
                        case SCORE3:
                            follower.followPath(score3);
                            break;
                        case SCORE4:
                            follower.followPath(score4);
                            break;
                    }
                    CS = STATES.SPECIMEN;
                    break;

                case WALL:
                    follower.followPath(wall, false);
                    robotSystems.collectSpecimen();
                    lift.goToGround();

                    switch (SCORING_CS) {
                        case SCORE1:
                            SCORING_CS = SCORING_STATES.SCORE2;
                            break;
                        case SCORE2:
                            SCORING_CS = SCORING_STATES.SCORE3;
                            break;
                        case SCORE3:
                            SCORING_CS = SCORING_STATES.SCORE4;
                            break;
                    }
                    CS = STATES.MOVING;
                    NS = STATES.COLLECTING_SPECIMEN;
                    break;

                case PARK:
                    follower.followPath(park);
                    lift.goToGround();
                    outtakeSubsystem.goToTransfer();
                    CS = STATES.MOVING;
                    NS = STATES.PARKED;
                    break;

                case PARKED:
                    break;
            }

            follower.update();
            robotSystems.update();
            telemetry.addData("STATE", CS);
            telemetry.addData("TIMER", timer.milliseconds());
            telemetry.addData("MATCH TIMER", matchTimer.seconds());
            telemetry.update();

        }

    }
}

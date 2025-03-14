package htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
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
@Autonomous(name = "[AUTO] 0 + 5", group = "HTECH")
public class AutoBasket extends LinearOpMode {

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
    Path sample1, sample2, sample3, sampleHuman;
    Path score1, score2, score3, scoreSampleHuman;
    Path park;


    //States
    public enum STATES{
        IDLE,
        GOING_TO_PRELOAD,
        SCORING_PRELOAD,
        OPEN_CLAW,
        MOVING, WAITING, TRANSFERING,
        SAMPLE1, SAMPLE2, SAMPLE3,
        SCORE1, SCORE2, SCORE3,
        SAMPLE_HUMAN, SCORE_SAMPLE_HUMAN, COLLECT_SAMPLE, COLLECT_SAMPLE2, COLLECT_SAMPLE3,
        SCORE_SAMPLE_HUMAN2, SCORE_SAMPLE_HUMAN3,
        PARK, PARKING,
        FLICK_BAR
    }

    public enum SAMPLE_STATES{
        PRELOAD,
        SAMPLE1,
        SAMPLE2,
        SAMPLE3,
        SAMPLE_HUMAN
    }

    STATES CS = STATES.IDLE, NS = STATES.IDLE;
    SAMPLE_STATES SAMPLE_CS = SAMPLE_STATES.PRELOAD;

    //Coordinates
    public static double startX = 137, startY = 36, startH = 90;

    public static double scoreX = 127, scoreY = 22.4, scoreH = 125;
    public static double scoreHumanX = 129, scoreHumanY = 26.5, scoreHumanH = 125;

    public static double sample1X = 122.6, sample1Y = 23.4, sample1H = 180;
    public static double sample2X = 122, sample2Y = 15.6, sample2H = 180;
    public static double sample3X = 119.8, sample3Y = 13.5, sample3H = 200;

    public static double sampleHumanX = 135, sampleHumanY = 90, sampleHumanH = 90;

    public static double parkX = 85.4, parkY = 51, parkH = 90;
    public static double parkSafeX = 84, parkSafeY = 27;


    //Booleans
    boolean firstSample = false;
    boolean secondSample = false;
    boolean thirdSample = false;
    boolean fourthSample = false;
    boolean basket = false;
    boolean parkkk = false;

    //T values
    public static double sample1T = 0.2;
    public static double sample2T = 0.4;
    public static double sample3T = 0.4;
    public static double sample4T = 0.4;
    public static double parkT = 0.3;


    //Timers
    public static double matchTime = 30;
    public static double timeToWait = 0;
    public static double timeToFlickBar = 300;
    public static double timeToToggleClaw = 100;
    public static double timeToCollectSample = 200;
    public static double timeToGoDownSample = 350;
    public static double timeToExtendSlides = 650;


    //Lift positions
    public static int liftPosPark = 400;

    //extendo poses
    public static int extendPos1 = 350;


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

        intakeSubsystem.goToMoving();
        outtakeSubsystem.bar.init();
        outtakeSubsystem.funny.retract();
        outtakeSubsystem.claw.close();
        extendo.pidEnabled = true;
        follower.setMaxPower(1);

        preload = new Path(
                new BezierLine(
                        new Point(startX, startY, Point.CARTESIAN),
                        new Point(scoreX, scoreY, Point.CARTESIAN)
                )
        );
        preload.setLinearHeadingInterpolation(Math.toRadians(startH), Math.toRadians(scoreH));

        sampleHuman = new Path(
                new BezierLine(
                        new Point(scoreX, scoreY, Point.CARTESIAN),
                        new Point(sampleHumanX, sampleHumanY, Point.CARTESIAN)
                )
        );
        sampleHuman.setLinearHeadingInterpolation(Math.toRadians(scoreH), Math.toRadians(sampleHumanH));

        sample1 = new Path(
                new BezierLine(
                        new Point(scoreX, scoreY, Point.CARTESIAN),
                        new Point(sample1X, sample1Y, Point.CARTESIAN)
                )
        );
        sample1.setLinearHeadingInterpolation(Math.toRadians(scoreH), Math.toRadians(sample1H));
        sample1.setPathEndTimeoutConstraint(350);

        sample2 = new Path(
                new BezierLine(
                        new Point(scoreX, scoreY, Point.CARTESIAN),
                        new Point(sample2X, sample2Y, Point.CARTESIAN)
                )
        );
        sample2.setLinearHeadingInterpolation(Math.toRadians(scoreH), Math.toRadians(sample2H));
        sample2.setPathEndTimeoutConstraint(350);

        sample3 = new Path(
                new BezierLine(
                        new Point(scoreX, scoreY, Point.CARTESIAN),
                        new Point(sample3X, sample3Y, Point.CARTESIAN)
                )
        );
        sample3.setLinearHeadingInterpolation(Math.toRadians(scoreH), Math.toRadians(sample3H));
        sample3.setPathEndTimeoutConstraint(350);

        scoreSampleHuman = new Path(
                new BezierLine(
                        new Point(sampleHumanX, sampleHumanY, Point.CARTESIAN),
                        new Point(scoreHumanX, scoreHumanY, Point.CARTESIAN)
                )
        );
        scoreSampleHuman.setLinearHeadingInterpolation(Math.toRadians(sampleHumanH), Math.toRadians(scoreHumanH));

        score1 = new Path(
                new BezierLine(
                        new Point(sample1X, sample1Y, Point.CARTESIAN),
                        new Point(scoreX, scoreY, Point.CARTESIAN)
                )
        );
        score1.setLinearHeadingInterpolation(Math.toRadians(sample1H), Math.toRadians(scoreH));

        score2 = new Path(
                new BezierLine(
                        new Point(sample2X, sample2Y, Point.CARTESIAN),
                        new Point(scoreX, scoreY, Point.CARTESIAN)
                )
        );
        score2.setLinearHeadingInterpolation(Math.toRadians(sample2H), Math.toRadians(scoreH));

        score3 = new Path(
                new BezierLine(
                        new Point(sample3X, sample3Y, Point.CARTESIAN),
                        new Point(scoreX, scoreY, Point.CARTESIAN)
                )
        );
        score3.setLinearHeadingInterpolation(Math.toRadians(sample3H), Math.toRadians(scoreH));

        park = new Path(
                new BezierCurve(
                        new Point(scoreX, scoreY, Point.CARTESIAN),
                        new Point(parkSafeX, parkSafeY, Point.CARTESIAN),
                        new Point(parkX, parkY, Point.CARTESIAN)
                )
        );
        park.setLinearHeadingInterpolation(Math.toRadians(scoreH), Math.toRadians(parkH));
        park.setPathEndTimeoutConstraint(500);

        waitForStart();

        matchTimer.reset();

        while(opModeIsActive() && matchTimer.seconds() < matchTime){

            switch (CS){

                case IDLE:
                    intakeSubsystem.goToWall();
                    CS = STATES.GOING_TO_PRELOAD;
                    break;

                case GOING_TO_PRELOAD:
                    follower.followPath(preload);
                    lift.goToHighBasket();
                    CS = STATES.MOVING;
                    NS = STATES.SCORING_PRELOAD;
                    break;

                case MOVING:
                    if(basket){
                        if(robotSystems.transferState == RobotSystems.TransferStates.IDLE){
                            lift.goToHighBasket();
                            basket = false;
                        }
                    }
                    if(firstSample){
                        if(follower.getCurrentTValue() > sample1T){
                            lift.goToGround();
                            firstSample = false;
                        }
                    }
                    if(secondSample){
                        if(follower.getCurrentTValue() > sample2T){
                            lift.goToGround();
                            secondSample = false;
                        }
                    }
                    if(thirdSample){
                        if(follower.getCurrentTValue() > sample3T){
                            lift.goToGround();
                            thirdSample = false;
                        }
                    }
                    if(fourthSample){
                        if(follower.getCurrentTValue() > sample4T){
                            lift.goToGround();
                            fourthSample = false;
                        }
                    }
                    if(parkkk){
                        if(follower.getCurrentTValue() > parkT){
                            lift.goToHighChamber();
                            outtakeSubsystem.funny.extend();
                            outtakeSubsystem.bar.goToSpecimenScore();
                            parkkk = false;
                        }
                    }
                    if(!follower.isBusy()){
                        CS = NS;
                        timer.reset();
                    }
                    break;

                case WAITING:
                    if(timer.milliseconds() > timeToWait){
                        CS = NS;
                        timer.reset();
                    }
                    break;

                case SCORING_PRELOAD:
                    if(lift.isAtPosition()){
                        outtakeSubsystem.goToSampleScore();
                        CS = STATES.OPEN_CLAW;
                        timer.reset();
                    }
                    break;

                case FLICK_BAR:
                    outtakeSubsystem.bar.goToTransfer();
                    CS = STATES.WAITING;
                    switch (SAMPLE_CS){
                        case PRELOAD:
                            NS = STATES.SAMPLE_HUMAN;
                            break;
                        case SAMPLE_HUMAN:
                            NS = STATES.SAMPLE1;
                            break;
                        case SAMPLE1:
                            NS = STATES.SAMPLE2;
                            break;
                        case SAMPLE2:
                            NS = STATES.SAMPLE3;
                            break;
                        case SAMPLE3:
                            NS = STATES.PARK;
                            break;
                    }
                    timeToWait = timeToFlickBar;
                    timer.reset();
                    break;

                case OPEN_CLAW:
                    if(timer.milliseconds() > timeToFlickBar){
                        outtakeSubsystem.claw.open();
                        CS = STATES.WAITING;
                        NS = STATES.FLICK_BAR;
                        timeToWait = timeToToggleClaw;
                        timer.reset();
                    }
                    break;

                case SAMPLE_HUMAN:
                    follower.followPath(sampleHuman);
                    SAMPLE_CS = SAMPLE_STATES.SAMPLE_HUMAN;
                    firstSample = true;
                    CS = STATES.MOVING;
                    NS = STATES.COLLECT_SAMPLE;
                    break;

                case COLLECT_SAMPLE:
                    extendo.goToPos(extendPos1);
                    intakeSubsystem.goDown();
                    intakeSubsystem.claw.open();
                    if(SAMPLE_CS == SAMPLE_STATES.SAMPLE3){
                        intakeSubsystem.rotation.goToAutoPos();
                    }
                    if(extendo.isAtPosition()){
                        timeToWait = timeToCollectSample;
                        CS = STATES.WAITING;
                        NS = STATES.COLLECT_SAMPLE2;
                        timer.reset();
                    }
                    break;

                case COLLECT_SAMPLE2:
                    intakeSubsystem.bar.goToCollect();
                    if(timer.milliseconds() > timeToGoDownSample){
                        intakeSubsystem.claw.close();
                        CS = STATES.WAITING;
                        NS = STATES.COLLECT_SAMPLE3;
                        timeToWait = timeToToggleClaw;
                        timer.reset();
                    }
                    break;

                case COLLECT_SAMPLE3:
                    robotSystems.transfer();
                    switch (SAMPLE_CS){
                        case SAMPLE_HUMAN:
                            CS = STATES.SCORE_SAMPLE_HUMAN;
                            break;
                        case SAMPLE1:
                            CS = STATES.SCORE1;
                            break;
                        case SAMPLE2:
                            CS = STATES.SCORE2;
                            break;
                        case SAMPLE3:
                            CS = STATES.SCORE3;
                            break;
                    }
                    break;

                case SCORE_SAMPLE_HUMAN:
                    follower.setMaxPower(0.8);
                    follower.followPath(scoreSampleHuman);
                    CS = STATES.MOVING;
                    NS = STATES.SCORE_SAMPLE_HUMAN2;
                    basket = true;
                    break;

                case SCORE_SAMPLE_HUMAN2:
                    outtakeSubsystem.goToSampleScore();
                    CS = STATES.SCORE_SAMPLE_HUMAN3;
                    timer.reset();
                    break;

                case SCORE_SAMPLE_HUMAN3:
                    if(timer.milliseconds() > 100){
                        outtakeSubsystem.claw.open();
                        CS = STATES.SAMPLE1;
                        timer.reset();
                    }
                    break;

                case SAMPLE1:
                    follower.setMaxPower(1);
                    if(timer.milliseconds() > timeToToggleClaw){
                        follower.followPath(sample1, true);
                        SAMPLE_CS = SAMPLE_STATES.SAMPLE1;
                        secondSample = true;
                        CS = STATES.MOVING;
                        NS = STATES.COLLECT_SAMPLE;
                    }
                    break;

                case SCORE1:
                    follower.followPath(score1);
                    if(robotSystems.transferState == RobotSystems.TransferStates.IDLE){
                        lift.goToHighBasket();
                        if(lift.isAtPosition()){
                            outtakeSubsystem.goToSampleScore();
                            CS = STATES.OPEN_CLAW;
                            timer.reset();
                        }
                    }
                    break;

                case SAMPLE2:
                    follower.followPath(sample2, true);
                    SAMPLE_CS = SAMPLE_STATES.SAMPLE2;
                    thirdSample = true;
                    CS = STATES.MOVING;
                    NS = STATES.COLLECT_SAMPLE;
                    break;

                case SCORE2:
                    follower.followPath(score2);
                    if(robotSystems.transferState == RobotSystems.TransferStates.IDLE){
                        lift.goToHighBasket();
                        if(lift.isAtPosition()){
                            outtakeSubsystem.goToSampleScore();
                            CS = STATES.OPEN_CLAW;
                            timer.reset();
                        }
                    }
                    break;

                case SAMPLE3:
                    follower.followPath(sample3, true);
                    SAMPLE_CS = SAMPLE_STATES.SAMPLE3;
                    fourthSample = true;
                    CS = STATES.MOVING;
                    NS = STATES.COLLECT_SAMPLE;
                    break;

                case SCORE3:
                    follower.followPath(score3);
                    if(robotSystems.transferState == RobotSystems.TransferStates.IDLE){
                        lift.goToHighBasket();
                        if(lift.isAtPosition()){
                            outtakeSubsystem.goToSampleScore();
                            CS = STATES.OPEN_CLAW;
                            timer.reset();
                        }
                    }
                    break;

                case PARK:
                    follower.followPath(park, true);
                    parkkk = true;
                    CS = STATES.MOVING;
                    NS = STATES.PARKING;
                    break;

                case PARKING:
                    follower.holdPoint(new Pose(parkX, parkY, Math.toRadians(parkH)));
                    lift.goToPos(liftPosPark);
                    outtakeSubsystem.bar.goToPark();
                    //matchTime = 0;
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

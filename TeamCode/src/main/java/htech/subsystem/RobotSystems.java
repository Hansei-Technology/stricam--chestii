package htech.subsystem;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import htech.config.PositionsExtendo;
import htech.config.RobotSettings;

public class RobotSystems {
    public ExtendoSystem extendoSystem;
    public LiftSystem liftSystem;
    public IntakeSubsystem intakeSubsystem;
    public OuttakeSubsystem outtakeSubsystem;
    public ElapsedTime timer;
    public ElapsedTime timerCollect;
    public ElapsedTime timerScore;
    public ElapsedTime timerTransfer;

    public boolean transferFirstTime = true;

    public boolean autoSample = false;
    public boolean fastCollect = false;
    public boolean caca = false;

    public RobotSystems(ExtendoSystem extendoSystem, LiftSystem liftSystem, IntakeSubsystem intakeSubsystem, OuttakeSubsystem outtakeSubsystem) {
        this.extendoSystem = extendoSystem;
        this.liftSystem = liftSystem;
        this.intakeSubsystem = intakeSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        timer = new ElapsedTime();
        timerCollect = new ElapsedTime();
        timerScore = new ElapsedTime();
        timerTransfer = new ElapsedTime();
    }

    public void update() {
        extendoSystem.update();
        liftSystem.update();
        intakeSubsystem.update();
        updateTransfer();
        updateCollectSpecimen();
        updateScoreSpecimen();
        updateCollect();

        if(!extendoSystem.pidEnabled && extendoSystem.currentPos > 150 && intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.WALL) intakeSubsystem.goDownWithoutResetRotation();
        else if(extendoSystem.pidEnabled && extendoSystem.target_position == PositionsExtendo.max && intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.WALL && extendoSystem.currentPos > 200) intakeSubsystem.goDown();
    }

    public enum TransferStates {
        IDLE,
        LIFT_GOING_DOWN,
        WAITING_FOR_INTAKE_ROTATION,
        INTAKE_GOING_TO_POSITION,
        OUTTAKE_CLAW_CLOSING,
        INTAKE_CLAW_OPENING,
        GOING_TO_AFTER_TRANSFER
    }

    public TransferStates transferState = TransferStates.IDLE;

    public enum collectSpecimenStates{
        IDLE,
        GETTING_IN_POSITION,
        WAITING,
        MAX_RETRACT_FUNNY,
        CLOSING_CLAW,
        WAITING_FOR_CLAW,
        GO_TO_SCORE,
    }
    public collectSpecimenStates collectSpecimenState = collectSpecimenStates.IDLE;

    public enum scoreSpecimenStates{
        IDLE,
        GO_TO_SCORE,
        WAITING_TO_SCORE,
        OPEN_CLAW,
        WAITING_TO_OPEN_CLAW
    }
    public scoreSpecimenStates scoreSpecimenState = scoreSpecimenStates.IDLE;



    public void transfer() {
        transferState = TransferStates.LIFT_GOING_DOWN;
        collectSpecimenState = collectSpecimenStates.IDLE;
        scoreSpecimenState = scoreSpecimenStates.IDLE;
        transferFirstTime = true;
        timerTransfer.reset();
    }

    public void updateTransfer() {
        switch (transferState) {
            case LIFT_GOING_DOWN:
                if(transferFirstTime) {
                    liftSystem.goToGround();
                    extendoSystem.goToGround();
                    outtakeSubsystem.goToTransfer();
                    intakeSubsystem.goToWall();
                    intakeSubsystem.rotation.goToNormal();
                    transferFirstTime = false;
                }
                if(liftSystem.isDown() && timerTransfer.milliseconds() > RobotSettings.outtake_going_to_transfer && extendoSystem.currentPos < 100) {
                    if(intakeSubsystem.rotation.rotLevel > 3) {
                        transferState = TransferStates.WAITING_FOR_INTAKE_ROTATION;
                    } else {
                        transferState = TransferStates.INTAKE_GOING_TO_POSITION;
                    }
                    transferFirstTime = true;
                    timerTransfer.reset();
                }
                break;
            case WAITING_FOR_INTAKE_ROTATION:
                if(timerTransfer.milliseconds() > RobotSettings.rotation_max_time) {
                    transferState = TransferStates.INTAKE_GOING_TO_POSITION;
                    timerTransfer.reset();
                    transferFirstTime = true;
                }
                break;
            case INTAKE_GOING_TO_POSITION:
                if(transferFirstTime) {
                    intakeSubsystem.goToTransfer();
                    transferFirstTime = false;
                }
                if(timerTransfer.milliseconds() > RobotSettings.moving_intake) {
                    transferState = TransferStates.OUTTAKE_CLAW_CLOSING;
                    timerTransfer.reset();
                    transferFirstTime = true;
                }
                break;
            case OUTTAKE_CLAW_CLOSING:
                outtakeSubsystem.claw.close();
                if(transferFirstTime) {

                    transferFirstTime = false;
                }
                if(timerTransfer.milliseconds() > RobotSettings.outtake_claw_close) {
                    transferState = TransferStates.INTAKE_CLAW_OPENING;
                    timerTransfer.reset();
                    transferFirstTime = true;
                }
                break;
            case INTAKE_CLAW_OPENING:
                if(transferFirstTime) {
                    intakeSubsystem.claw.open();
                    transferFirstTime = false;
                }
                if(timerTransfer.milliseconds() > RobotSettings.intake_claw_open) {
                    transferState = TransferStates.GOING_TO_AFTER_TRANSFER;
                    timerTransfer.reset();
                    transferFirstTime = true;
                }
                break;
            case GOING_TO_AFTER_TRANSFER:
                if(transferFirstTime) {
                    outtakeSubsystem.goToAfterTransfer();
                    intakeSubsystem.goToWall();
                    transferFirstTime = false;
                }
                if(timerTransfer.milliseconds() > RobotSettings.going_after_transfer) {
                    transferState = TransferStates.IDLE;
                }
                break;
        }
    }


    public boolean isTransfering(){
        return transferState != TransferStates.IDLE;
    }

    public void updateCollectSpecimen(){

        switch (collectSpecimenState) {

            case IDLE:
                break;

            case GETTING_IN_POSITION:
                liftSystem.goToGround();
                outtakeSubsystem.goToPreCollectSpecimen();
                collectSpecimenState = collectSpecimenStates.WAITING;
                break;

            case WAITING:
                break;

            case MAX_RETRACT_FUNNY:
                outtakeSubsystem.funny.maxRetract();
                collectSpecimenState = collectSpecimenStates.CLOSING_CLAW;
                timerCollect.reset();
                break;

            case CLOSING_CLAW:
                if(timerCollect.milliseconds() > RobotSettings.timeToToggleFunny){
                    outtakeSubsystem.claw.close();
                    collectSpecimenState = collectSpecimenStates.WAITING_FOR_CLAW;
                    timerCollect.reset();
                }
                break;

            case WAITING_FOR_CLAW:
                if(timerCollect.milliseconds() > RobotSettings.timeToCloseClaw){
                    collectSpecimenState = collectSpecimenStates.GO_TO_SCORE;
                }
                break;

            case GO_TO_SCORE:
                liftSystem.goToHighChamber();
                outtakeSubsystem.goToSpecimenPrescore();
                collectSpecimenState = collectSpecimenStates.IDLE;
                break;
        }

    }

    public void collectSpecimen(){
        collectSpecimenState = collectSpecimenStates.GETTING_IN_POSITION;
    }

    public void updateScoreSpecimen(){

        switch (scoreSpecimenState){

            case IDLE:
                break;

            case GO_TO_SCORE:
                outtakeSubsystem.goToSpecimenScore();
                liftSystem.goToScoreSpecimen();
                scoreSpecimenState = scoreSpecimenStates.WAITING_TO_SCORE;
                timerScore.reset();
                break;

            case WAITING_TO_SCORE:
                if(timerScore.milliseconds() > RobotSettings.outtake_score){
                    scoreSpecimenState = scoreSpecimenStates.OPEN_CLAW;
                }
                break;

            case OPEN_CLAW:
                outtakeSubsystem.claw.open();
                scoreSpecimenState = scoreSpecimenStates.IDLE;
                collectSpecimenState = collectSpecimenStates.GETTING_IN_POSITION;
                break;
        }

    }

    public void updateCollect() {
        switch (intakeSubsystem.intakeState) {
            case COLLECT_GOING_DOWN:
                if(!intakeSubsystem.claw.isOpen) {
                    intakeSubsystem.intakeState = IntakeSubsystem.IntakeState.COLLECTING;
                    timer.reset();

                }
                break;
            case COLLECTING:
                if(timer.milliseconds() > RobotSettings.intake_claw_open) {
                    intakeSubsystem.intakeState = IntakeSubsystem.IntakeState.COLECT_GOING_UP;
                    intakeSubsystem.bar.goToGround();
                    timer.reset();
                }
                break;
            case COLECT_GOING_UP:
                if(timer.milliseconds() > RobotSettings.intake_move_collect) {
                    if(intakeSubsystem.hasElement()) {
                        intakeSubsystem.goToWall();
                        extendoSystem.goToPos(PositionsExtendo.beforeTransfer);
                    } else {
                        intakeSubsystem.goDownWithoutResetRotation();
                    }
                }
        }
    }

    public void scoreSpecimen(){
        scoreSpecimenState = scoreSpecimenStates.GO_TO_SCORE;
        collectSpecimenState = collectSpecimenStates.IDLE;
    }



}

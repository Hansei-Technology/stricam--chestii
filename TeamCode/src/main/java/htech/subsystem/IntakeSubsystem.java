package htech.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import htech.mechanism.intake.BreakBeam;
import htech.mechanism.intake.ColorSensor;
import htech.mechanism.intake.IntakeBar;
import htech.mechanism.intake.IntakeClaw;
import htech.mechanism.intake.IntakeJoint;
import htech.mechanism.intake.IntakeRotation;

@Config
public class IntakeSubsystem {
    public final IntakeClaw claw;
    public final IntakeRotation rotation;
    public final IntakeBar bar;
    public final IntakeJoint joint;

    ColorSensor colorSensor;

    boolean fastCollect = false;

    public enum IntakeState {
        DOWN,
        WALL,
        READY,
        TRANSFER,
        COLLECT_GOING_DOWN,
        COLLECTING,
        COLECT_GOING_UP
    }
    public IntakeState intakeState = IntakeState.DOWN;




    public IntakeSubsystem(HardwareMap hardwareMap) {
        // MECHANISM //
        colorSensor = new ColorSensor(hardwareMap);
        claw = new IntakeClaw(hardwareMap);
        rotation = new IntakeRotation(hardwareMap);
        bar = new IntakeBar(hardwareMap);
        joint = new IntakeJoint(hardwareMap);
    }

    public void goToLimeLight() {
        joint.goToWall();
        bar.goToLimeLight();
        rotation.goToFlipped();
        claw.open();
    }

    public void init() {
        joint.goToPickup();
        bar.goToGround();
        rotation.goToFlipped();
        claw.open();
    }

    public boolean hasElement() {
        return colorSensor.hasElement();
    }

    public void initAuto() {
        joint.goToReady();
        bar.goToTransfer();
        rotation.goToFlipped();
        claw.open();
    }

    public void goDown() {
        joint.goToPickup();
        bar.goToGround();
        rotation.rotLevel = 0;
        rotation.goToFlipped();
//        claw.open();
        if(intakeState != intakeState.COLLECTING) intakeState = intakeState.DOWN;
    }

    public void goDownWithoutResetRotation() {
        joint.goToPickup();
        bar.goToGround();
        claw.open();
        //if(intakeState != intakeState.COLLECTING)
        intakeState = intakeState.DOWN;
    }

    public void goToMoving(){
        bar.goToMoving();
        joint.goToMoving();
        claw.close();
    }

    public void goToWall() {
        joint.goToWall();
        bar.goToWall();
        rotation.goToFlipped();
        intakeState = intakeState.WALL;
    }

    public void goToReady() {
        joint.goToReady();
        bar.goToReady();
        rotation.goToNormal();
        intakeState = intakeState.READY;
    }

    public void goToTransfer() {
        joint.goToTransfer();
        bar.goToTransfer();
        rotation.goToNormal();
        intakeState = intakeState.TRANSFER;
    }

    public void collect() {
        if(intakeState == intakeState.DOWN) {
            joint.goToCollect();
            bar.goToCollect();
            intakeState = intakeState.COLLECT_GOING_DOWN;
        }
        else if(intakeState == IntakeState.WALL) goDown();
        else goDownWithoutResetRotation();
    }

    public void update() {
        bar.update();
        joint.update();
        rotation.update();
        claw.update();
    }

    public void goToCollectSubmersible() {
        joint.goToPickup();
        bar.goToCollectSub();
        intakeState = intakeState.DOWN;
    }

    public void goToSpecialTransfer() {
        joint.goToSpecialTransfer();
        bar.goToSpecialTransfer();
        rotation.goToNormal();
        intakeState = intakeState.TRANSFER;
    }
}


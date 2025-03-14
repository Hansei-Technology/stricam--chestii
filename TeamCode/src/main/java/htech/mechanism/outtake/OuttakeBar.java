package htech.mechanism.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import htech.config.PositionsOuttake;
import htech.config.Servos;

public class OuttakeBar {
    private final Servo servoLeft;
    private final Servo servoRight;

    private double currentPositionLeft;
    private double currentPositionRight;



    public OuttakeBar(HardwareMap hardwareMap) {
        servoLeft = hardwareMap.get(Servo.class, Servos.outtakeLeft);
        servoRight = hardwareMap.get(Servo.class, Servos.outtakeRight);
//
//        servoLeft.setPosition(PositionsOuttake.transferBar);
//        servoRight.setPosition(PositionsOuttake.transferBar);
//
//        currentPositionLeft = PositionsOuttake.transferBar;
//        currentPositionRight = PositionsOuttake.transferBar;
    }

    public void goToTransfer() {
        servoLeft.setPosition(PositionsOuttake.transferBar);
        servoRight.setPosition(PositionsOuttake.transferBar);

        currentPositionLeft = PositionsOuttake.transferBar;
        currentPositionRight = PositionsOuttake.transferBar;
    }

    public void goToPark(){
        servoLeft.setPosition(PositionsOuttake.parkBar);
        servoRight.setPosition(PositionsOuttake.parkBar);

        currentPositionLeft = PositionsOuttake.parkBar;
        currentPositionRight = PositionsOuttake.parkBar;
    }

    public void init(){
        servoLeft.setPosition(PositionsOuttake.initBar);
        servoRight.setPosition(PositionsOuttake.initBar);

        currentPositionLeft = PositionsOuttake.initBar;
        currentPositionRight = PositionsOuttake.initBar;
    }

    public void goToSpecimenScore() {
        servoLeft.setPosition(PositionsOuttake.specimenBar);
        servoRight.setPosition(PositionsOuttake.specimenBar);

        currentPositionLeft = PositionsOuttake.specimenBar;
        currentPositionRight = PositionsOuttake.specimenBar;
    }

    public void goToSpecimenPrescore(){
        servoLeft.setPosition(PositionsOuttake.specimenPrescoreBar);
        servoRight.setPosition(PositionsOuttake.specimenPrescoreBar);

        currentPositionLeft = PositionsOuttake.specimenPrescoreBar;
        currentPositionRight = PositionsOuttake.specimenPrescoreBar;
    }

    public void goToSample() {
        servoLeft.setPosition(PositionsOuttake.sampleBar);
        servoRight.setPosition(PositionsOuttake.sampleBar);

        currentPositionLeft = PositionsOuttake.sampleBar;
        currentPositionRight = PositionsOuttake.sampleBar;
    }

    public void goToSpecimenCollect() {
        servoLeft.setPosition(PositionsOuttake.specimenCollectBar);
        servoRight.setPosition(PositionsOuttake.specimenCollectBar);

        currentPositionLeft = PositionsOuttake.specimenCollectBar;
        currentPositionRight = PositionsOuttake.specimenCollectBar;
    }

    public void goToAfterTransfer() {
        servoLeft.setPosition(PositionsOuttake.afterTransferBar);
        servoRight.setPosition(PositionsOuttake.afterTransferBar);

        currentPositionLeft = PositionsOuttake.afterTransferBar;
        currentPositionRight = PositionsOuttake.afterTransferBar;
    }

}

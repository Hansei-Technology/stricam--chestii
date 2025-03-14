package htech.mechanism.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import htech.config.PositionsIntake;
import htech.config.Servos;

public class IntakeJoint {
    private final Servo jointServo;
    private double currentPosition;

    public IntakeJoint(HardwareMap hardwareMap) {
        jointServo = hardwareMap.get(Servo.class, Servos.intakeJointServo);
//        jointServo.setPosition(PositionsIntake.prepTransferPositionJoint);
//        currentPosition = PositionsIntake.prepTransferPositionJoint;
    }

    public void goToPickup() {
        jointServo.setPosition(PositionsIntake.groundPositionJoint);
        currentPosition = PositionsIntake.groundPositionJoint;
    }

    public void goToWall() {
        jointServo.setPosition(PositionsIntake.wallPickupPositionJoint);
        currentPosition = PositionsIntake.wallPickupPositionJoint;
    }
    public void goToReady() {
        jointServo.setPosition(PositionsIntake.prepTransferPositionJoint);
        currentPosition = PositionsIntake.prepTransferPositionJoint;
    }

    public void goToPreTransferSample() {
        jointServo.setPosition(PositionsIntake.readyPositionJointSample);
        currentPosition = PositionsIntake.readyPositionJointSample;
    }

    public void goToTransfer() {
        jointServo.setPosition(PositionsIntake.transferPositionJoint);
        currentPosition = PositionsIntake.transferPositionJoint;
    }

    public void goToTransferSample() {
        jointServo.setPosition(PositionsIntake.transferPositionJointSample);
        currentPosition = PositionsIntake.transferPositionJointSample;
    }

    public void goToMoving(){
        jointServo.setPosition(PositionsIntake.movingJoint);
        currentPosition = PositionsIntake.movingJoint;
    }

    public void goToCollect() {
        jointServo.setPosition(PositionsIntake.collectPositionJoint);
        currentPosition = PositionsIntake.collectPositionJoint;
    }

    public void goToSpecialTransfer() {
        jointServo.setPosition(PositionsIntake.specialTransferJoint);
        currentPosition = PositionsIntake.specialTransferJoint;
    }


    public void update() {
        jointServo.setPosition(currentPosition);
    }
    public boolean isAtPos(){
        return Math.abs(jointServo.getPosition() - currentPosition) < 0.01;
    }

}

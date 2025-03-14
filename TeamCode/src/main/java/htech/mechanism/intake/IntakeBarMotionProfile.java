package htech.mechanism.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import htech.classes.DualServoController;
import htech.config.PositionsIntake;
import htech.config.Servos;

public class IntakeBarMotionProfile {
    private final Servo barServo;
    private final Servo barServo2;
    private double currentPosition;
    DualServoController dualServoController;

    public IntakeBarMotionProfile(HardwareMap hardwareMap) {
        barServo = hardwareMap.get(Servo.class, Servos.intakeBarServoLeft);
        barServo2 = hardwareMap.get(Servo.class, Servos.intakeBarServoRight);
//        barServo.setPosition(PositionsIntake.transferPositionBar);
//        barServo2.setPosition(PositionsIntake.transferPositionBar);

        dualServoController = new DualServoController(barServo, barServo2);
        dualServoController.setTargetPosition(PositionsIntake.transferPositionBar);

//        currentPosition = PositionsIntake.transferPositionBar;
    }

    public void goToGround() {
        dualServoController.setTargetPosition(PositionsIntake.groundPositionBar);
        currentPosition = PositionsIntake.groundPositionBar;
    }

    public void goToWall() {
        dualServoController.setTargetPosition(PositionsIntake.wallPositionBar);
        currentPosition = PositionsIntake.wallPositionBar;
    }

    public void goToTransfer() {
        dualServoController.setTargetPosition(PositionsIntake.transferPositionBar);
        currentPosition = PositionsIntake.transferPositionBar;
    }

    public void cycleHeight() {
        if (currentPosition == PositionsIntake.groundPositionBar) {
            this.goToWall();
        } else if (currentPosition == PositionsIntake.wallPositionBar) {
            this.goToGround();
        }
    }

    public void goToCollect() {
        dualServoController.setTargetPosition(PositionsIntake.collectPositionBar);
        currentPosition = PositionsIntake.collectPositionBar;
    }

    public boolean isAtPos(){
        return Math.abs(barServo.getPosition() - currentPosition) < 0.01;
    }

    public void update() {
        dualServoController.update();
    }
}

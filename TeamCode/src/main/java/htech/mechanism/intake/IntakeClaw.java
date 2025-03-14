package htech.mechanism.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import htech.config.PositionsIntake;
import htech.config.Servos;

public class IntakeClaw {
    private final Servo clawServo;
    public boolean isOpen = false;

    public IntakeClaw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, Servos.intakeClawServo);
//        clawServo.setPosition(PositionsIntake.openedClaw);
    }

    public void open() {
        clawServo.setPosition(PositionsIntake.openedClaw);
        isOpen = true;
    }

    public void close() {
        clawServo.setPosition(PositionsIntake.closedClaw);
        isOpen = false;
    }

    public void toggle() {
        if (isOpen) {
            clawServo.setPosition(PositionsIntake.closedClaw);
        } else {
            clawServo.setPosition(PositionsIntake.openedClaw);
        }

        isOpen = !isOpen;
    }

    public void update() {
        //clawServo.setPosition(isOpen ? PositionsIntake.openedClaw : PositionsIntake.closedClaw);
    }
}

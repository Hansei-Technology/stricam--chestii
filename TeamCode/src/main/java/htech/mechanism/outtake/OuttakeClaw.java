package htech.mechanism.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import htech.config.PositionsIntake;
import htech.config.PositionsOuttake;
import htech.config.Servos;

public class OuttakeClaw {
    private final Servo clawServo;
    public boolean isOpen = false;

    public OuttakeClaw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, Servos.outtakeClaw);
//        clawServo.setPosition(PositionsOuttake.openedClaw);
    }

    public void open() {
        clawServo.setPosition(PositionsOuttake.openedClaw);
        isOpen = true;
    }

    public void close() {
        clawServo.setPosition(PositionsOuttake.closedClaw);
        isOpen = false;
    }

    public void toggle() {
        if (isOpen) {
            clawServo.setPosition(PositionsOuttake.closedClaw);
        } else {
            clawServo.setPosition(PositionsOuttake.openedClaw);
        }

        isOpen = !isOpen;
    }


}

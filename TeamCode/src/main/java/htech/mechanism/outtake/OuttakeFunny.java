package htech.mechanism.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import htech.config.PositionsOuttake;
import htech.config.Servos;

public class OuttakeFunny {
    Servo servo;

    public OuttakeFunny(HardwareMap map) {
        servo = map.get(Servo.class, Servos.outtakeFunny);
    }

    public void extend() {
        servo.setPosition(PositionsOuttake.extendedFunny);
    }

    public void retract() {
        servo.setPosition(PositionsOuttake.retractedFunny);
    }

    public void halfExtend() {
        servo.setPosition(PositionsOuttake.halfExtendedFunny);
    }

    public void maxRetract(){
        servo.setPosition(PositionsOuttake.maxRetractFunny);
    }

    public void goToTransfer(){
        servo.setPosition(PositionsOuttake.transferFunny);
    }
}

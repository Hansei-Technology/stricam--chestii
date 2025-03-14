package htech.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import htech.config.Servos;

@Config
public class HangSystem {
    public CRServo left, right;
    public int reversed = 1;

    public HangSystem(HardwareMap map) {
        left = map.get(CRServo.class, Servos.hangLeftServo);
        right = map.get(CRServo.class, Servos.hangRightServo);
    }

    public void setPower(double power) {
        left.setPower(-power);
        right.setPower(power);
    }
}

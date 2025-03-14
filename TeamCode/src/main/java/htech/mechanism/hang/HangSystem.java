package htech.mechanism.hang;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import htech.config.Servos;

@Config
public class HangSystem {
    public static boolean goesSameWay = true;
    private CRServo left, right;
    public HangSystem(HardwareMap hardwareMap) {
        left = hardwareMap.get(CRServo.class, Servos.hangLeftServo);
        right = hardwareMap.get(CRServo.class, Servos.hangRightServo);
    }

    public void setPower(double power) {
        left.setPower(power);
        right.setPower(goesSameWay? power : -power);
    }
}


package htech.mechanism.outtake;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import htech.config.Sensors;

public class LimitSwitch {
    public DigitalChannel sensor;

    public LimitSwitch(HardwareMap map) {
        sensor = map.get(DigitalChannel.class, Sensors.LimitSwitch);
        sensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isPressed() {
        return !sensor.getState();
    }
}

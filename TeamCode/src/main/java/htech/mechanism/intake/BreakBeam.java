package htech.mechanism.intake;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SerialNumber;

import htech.config.Sensors;

public class BreakBeam {
    DigitalChannel sensor;
    public BreakBeam(HardwareMap map) {
        sensor = map.get(DigitalChannel.class, Sensors.BreakBeamIntake);
        sensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean hasElement() {
        return !sensor.getState();
    }
}

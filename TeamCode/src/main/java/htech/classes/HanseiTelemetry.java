package htech.classes;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HanseiTelemetry {
    MultipleTelemetry telemetry;

    // CONSTRUCTOR
    public HanseiTelemetry(MultipleTelemetry telemetry) {
        this.telemetry = telemetry;
    }

    // METHODS

    public void status(String value) {
        telemetry.addLine("[STATUS] " + value);
    }

    public void info(String value) {
        telemetry.addLine("[INFO] " + value);
    }

    public void warning(String value) {
        telemetry.addLine("[WARNING] " + value);
    }

    public void error(String value) {
        telemetry.addLine("[ERROR] " + value);
    }

    public void success(String value) {
        telemetry.addLine("[SUCCESS] " + value);
    }

    public void clear() {
        telemetry.clearAll();
    }

    public void update() {
        telemetry.update();
    }
}

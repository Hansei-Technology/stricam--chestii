package htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing Servo assigned slots specified in the Control Hub.
 */
@Config
public abstract class Sensors {
    public static String BreakBeamIntake = "d7";
    public static String LimitSwitch = "d1e";
    public static String colorSensor = "colorv3";
}

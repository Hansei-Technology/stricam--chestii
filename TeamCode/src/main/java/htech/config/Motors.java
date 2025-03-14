package htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing DcMotorEx assigned slots specified in the Control Hub.
 */
@Config
public abstract class Motors {
    public static String rightFrontMotor = "m1";
    public static String leftFrontMotor = "m3";
    public static String leftRearMotor = "m2";
    public static String rightRearMotor = "m0";

    public static String lift1 = "m1e";
    public static String lift2 = "m0e";
    public static String extendo = "m2e";
    public static String liftEncoder = "m1";
}

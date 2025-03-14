package htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing Servo assigned slots specified in the Control Hub.
 */
@Config
public abstract class Servos {
    public static String outtakeLeft = "s2e";
    public static String outtakeRight = "s3e";

    public static String outtakeClaw = "s1e";

    public static String intakeBarServoLeft = "s1";
    public static String intakeBarServoRight = "s2";

    public static String intakeJointServo = "s3";
    public static String intakeRotationServo = "s5e";
    public static String intakeClawServo = "s4e";


    public static String hangLeftServo = "s5";
    public static String hangRightServo = "s0";

    public static String outtakeFunny = "s4";
}

package htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing the various subsystem dependent values, like speed.
 */
@Config
public abstract class RobotSettings {


    public static double speed = 1;
    public static double rotationSpeed = 0.8;

    //transfer timers
    public static int outtake_going_to_transfer = 200;
    public static int rotation_max_time = 150;
    public static int outtake_claw_close = 200;
    public static int intake_claw_open = 100;
    public static int going_after_transfer = 120;
    public static int moving_intake = 200;


    //collecting specimen timers
    public static double timeToCloseClaw = 150;
    public static double timeToToggleFunny = 100;

    //intake collect timers
    public static int intake_claw_close = 140;
    public static int intake_move_collect = 300;

    //specimen collect timers

    public static double limeLightXMultiplyer = 1;

    //specimen score timers
    public static int outtake_score = 300;

}

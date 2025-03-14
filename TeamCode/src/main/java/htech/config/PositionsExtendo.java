package htech.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PositionsExtendo {
    public static int ground = -5;
    public static int transfer = 0;
    public static int beforeTransfer = 20;
    public static int max = 425;
    public static int maxAuto = 345;
    public static int mid = 250;
    public static int limePoz = -30;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0.001;

    public static int maxLegal = 340;

    public static double freeSpeed = 0.4;
}

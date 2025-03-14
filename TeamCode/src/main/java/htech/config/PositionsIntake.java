package htech.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public abstract class PositionsIntake {
    // INTAKE CLAW //
    public static double closedClaw = 0.295;
    public static double closedClawSliding = 0.295;
    public static double openedClaw = 0.45;

    // INTAKE ROTATION //
//    public static double normalRotation = 0.275;
//    public static double perpendicularRotation = 0.56;
//    public static double flippedNormalRotation = 0.84;

    public static double normalRotation = 0.083;
    public static double perpendicularRotation = 0.3;
    public static double flippedNormalRotation = 0.642;
    public static double ticksPerDegree = 0.0031;

    public static double rotation30deg2 = 0.095;
    public static double rotation30Deg = 0.095;
    public static double rotSpeed = 0.15;
    public static double rotationAuto = 0.77;

    // INTAKE BAR //
    public static double groundPositionBar = 0.48; // over the samples
    public static double collectSubPositionBar = 0.45;
    public static double wallPositionBar = 0.465;
    public static double transferPositionBar = 0.68; // maybe change this
    public static double readyPositionBar = 0.725;
    public static double collectPositionBar = 0.41; // collect position = lower than ground
    public static double offsetBar = 0;
    public static double movingBar = 0.73;

    // INTAKE JOINT //
    public static double movingJoint = 0.55;
    public static double groundPositionJoint = 0.24;
    public static double wallPickupPositionJoint = 0.44;
    public static double prepTransferPositionJoint = 0.68;
    public static double transferPositionJoint = 0.7;
    public static double collectPositionJoint = 0.24;

    //sample positions for joint
    public static double transferPositionBarSample = 0.755;
    public static double transferPositionJointSample = 0.76;
    public static double readyPositionBarSample = 0.71;
    public static double readyPositionJointSample = 0.73;

    public static double limeLightPositionBar = 0.6;

    public static float colorSensorGain = 2;
    public static float thresholdRed = 0.016f;
    public static float thresholdGreen = 0.03f;
    public static float thresholdBlue = 0.025f;


    public static double specialTranferBar = 0.9;
    public static double specialTransferJoint = 0.45;
}
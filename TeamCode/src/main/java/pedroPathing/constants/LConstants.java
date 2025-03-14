package pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class LConstants {
    public static double forwardY = -1.9;
    public static double strafeX = -2.2;
    static {
        PinpointConstants.forwardY = forwardY;
        PinpointConstants.strafeX = strafeX;
        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        PinpointConstants.hardwareMapName = "pinpoint";
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
        PinpointConstants.useCustomEncoderResolution = false;
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        PinpointConstants.customEncoderResolution = 13.26291192;
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;


//        TwoWheelConstants.forwardTicksToInches = .001962385587127942;
//        TwoWheelConstants.strafeTicksToInches = .0019706390586948404;
//        TwoWheelConstants.forwardY = forwardY;
//        TwoWheelConstants.strafeX = strafeX;
//        TwoWheelConstants.forwardEncoder_HardwareMapName = "m0e";
//        TwoWheelConstants.strafeEncoder_HardwareMapName = "m3e";
//        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
//        TwoWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
//        TwoWheelConstants.IMU_HardwareMapName = "imu";
//        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN);

    }
}





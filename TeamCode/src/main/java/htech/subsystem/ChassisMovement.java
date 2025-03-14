package htech.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import htech.config.Motors;
import htech.config.RobotSettings;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class ChassisMovement {
    private final DcMotorEx leftFrontMotor;
    private final DcMotorEx rightFrontMotor;
    private final DcMotorEx leftRearMotor;
    private final DcMotorEx rightRearMotor;
//    private final Follower follower;

    private static final double speed = RobotSettings.speed;
    private static final double rotationSpeed = RobotSettings.rotationSpeed;

    public ChassisMovement(HardwareMap hardwareMap) {
        // MOTOR DECLARATION //
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, Motors.leftFrontMotor);
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, Motors.rightFrontMotor);
        leftRearMotor = hardwareMap.get(DcMotorEx.class, Motors.leftRearMotor);
        rightRearMotor = hardwareMap.get(DcMotorEx.class, Motors.rightRearMotor);

//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(hardwareMap);

        // MOTOR CONFIGURATION //
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotorEx.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        List<DcMotor> motors = Arrays.asList(this.leftFrontMotor, this.leftRearMotor, this.rightFrontMotor, this.rightRearMotor);
        for (DcMotor dcMotor : motors) {
            DcMotorEx motor = (DcMotorEx) dcMotor;
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
    }

    public void updateMovementSlowRotation(Gamepad g) {
        leftFrontMotor.setPower((-g.left_stick_y + g.left_stick_x + g.right_stick_x * 0.75) * speed);
        rightFrontMotor.setPower((-g.left_stick_y - g.left_stick_x - g.right_stick_x * 0.75) * speed);
        leftRearMotor.setPower((-g.left_stick_y - g.left_stick_x + g.right_stick_x * 0.75) * speed);
        rightRearMotor.setPower((-g.left_stick_y + g.left_stick_x - g.right_stick_x * 0.75) * speed);
    }
}

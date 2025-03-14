package htech.subsystem;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import htech.classes.PIDController;
import htech.config.Motors;
import htech.config.PositionsIntake;
import htech.config.PositionsLift;

//2 motor lift system with PID
public class LiftSystem {
    private final DcMotorEx left, right;
//    private DcMotorEx encoder;
    public int target_position = 0;
    public PIDController pidController;
    public int currentPos = 0;
    public boolean PIDON = true;

    public LiftSystem(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, Motors.lift2);
        right = hardwareMap.get(DcMotorEx.class, Motors.lift1);
//        encoder = hardwareMap.get(DcMotorEx.class, Motors.liftEncoder);

//        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

//        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        right.setDirection(DcMotorEx.Direction.REVERSE);
        left.setDirection(DcMotorEx.Direction.FORWARD);

        MotorConfigurationType leftConfigurationType = left.getMotorType().clone();
        leftConfigurationType.setAchieveableMaxRPMFraction(1.0);
        left.setMotorType(leftConfigurationType);

        MotorConfigurationType rightConfigurationType = right.getMotorType().clone();
        rightConfigurationType.setAchieveableMaxRPMFraction(1.0);
        right.setMotorType(rightConfigurationType);

        pidController = new PIDController(PositionsLift.kP, PositionsLift.kI, PositionsLift.kD);
        pidController.targetValue = target_position;
        pidController.maxOutput = 1;
    }

    public void setPower(double power) {
        if(Math.abs(power) < 0.1) {
            power = 0;
        }
        left.setPower(power);
        right.setPower(power);
    }

    public void goToGround() {
        PIDON = true;
        target_position = PositionsLift.ground;
        pidController.targetValue = target_position;
    }

    public void goToHighChamber() {
        PIDON = true;
        target_position = PositionsLift.highChamber;
        pidController.targetValue = target_position;
    }

    public void goToScoreSpecimen() {
        PIDON = true;
        target_position = PositionsLift.scoreSpecimen;
        pidController.targetValue = target_position;
    }


    public void goToHighBasket() {
        PIDON = true;
        target_position = PositionsLift.highBasket;
        pidController.targetValue = target_position;
    }

    public void goToLowBasket() {
        PIDON = true;
        target_position = PositionsLift.lowBasket;
        pidController.targetValue = target_position;
    }

    public boolean isDown() {
        return currentPos < PositionsLift.ground + 25 && target_position == PositionsLift.ground;
    }

    public void goToPark() {
        target_position = PositionsLift.park;
        pidController.targetValue = target_position;
    }

    public void goToMinusPark() {
        target_position = -PositionsLift.park;
        pidController.targetValue = target_position;
    }

    public void reset() {
        left.setPower(0);
        right.setPower(0);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        currentPos = 0;
        target_position = 0;
        //g.rumble(100);
        PIDON = true;
    }



    public void goToPos(int position) {
        target_position = position;
        pidController.targetValue = target_position;
    }

    public boolean isAtPosition() {
        return Math.abs(target_position - currentPos) < 18;
    }

    public void update() {

        if(currentPos > PositionsLift.highChamber) {
            pidController.p = PositionsLift.kP2;
        } else {
            pidController.p = PositionsLift.kP;
        }

        if(PIDON) {
            currentPos = -right.getCurrentPosition();
            double power = pidController.update(currentPos);
            setPower(power);
        }

        if(pidController.p != PositionsLift.kP) {
            pidController.p = PositionsLift.kP;
        }
        if(pidController.i != PositionsLift.kI) {
            pidController.i = PositionsLift.kI;
        }
        if(pidController.d != PositionsLift.kD) {
            pidController.d = PositionsLift.kD;
        }
    }
}

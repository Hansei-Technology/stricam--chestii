package htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import htech.classes.StickyGamepad;
import htech.config.PositionsExtendo;
import htech.config.PositionsLift;
import htech.subsystem.ChassisMovement;
import htech.subsystem.ExtendoSystem;
import htech.subsystem.HangSystem;
import htech.subsystem.IntakeSubsystem;
import htech.subsystem.LiftSystem;
import htech.subsystem.OuttakeSubsystem;
import htech.subsystem.RobotSystems;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "[TELEOP] 1", group = "HTech")
public class TeleOp1 extends LinearOpMode {
    ChassisMovement chassisMovement;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    //HangSystem hang;
    ElapsedTime timer;
    ElapsedTime matchTimer;
    RobotSystems robotSystems;
    HangSystem hang;
    boolean pedroDrive = false;
    boolean reverseDrive = false;
    //    ChassisFollower chassisFollower;
    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        // SUBSYSTEMS //
        hang = new HangSystem(hardwareMap);
        chassisMovement = new ChassisMovement(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        lift = new LiftSystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);
        timer = new ElapsedTime();
        matchTimer = new ElapsedTime();
        robotSystems = new RobotSystems(extendo, lift, intakeSubsystem, outtakeSubsystem);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        chassisFollower = new ChassisFollower(hardwareMap);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // CLASSES //
        StickyGamepad stickyGamepad2 = new StickyGamepad(gamepad2, this);
        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1, this);
        waitForStart();

        intakeSubsystem.init();
        outtakeSubsystem.init();

        matchTimer.reset();

        while (opModeIsActive()) {
            hang.setPower(gamepad2.right_stick_y);


            chassisMovement.updateMovementSlowRotation(gamepad1);

            //intake
            if(stickyGamepad1.right_bumper) {
                if(lift.target_position == PositionsLift.highBasket)
                    outtakeSubsystem.claw.open();
                else if(lift.target_position == PositionsLift.highChamber) {
                    robotSystems.scoreSpecimen();
                } else if(robotSystems.scoreSpecimenState == RobotSystems.scoreSpecimenStates.IDLE) {
                    intakeSubsystem.collect();
                    robotSystems.collectSpecimenState = RobotSystems.collectSpecimenStates.IDLE;
                    outtakeSubsystem.goToTransfer();
                }
            }

            if(stickyGamepad1.x){
                robotSystems.transfer();//for specimen
            }
            if(stickyGamepad2.x) {
                robotSystems.transfer(); //for sample
            }

            if(gamepad1.a) {
                robotSystems.collectSpecimen();
                intakeSubsystem.goToWall();
            }

            if (stickyGamepad1.left_bumper) {
                if(robotSystems.collectSpecimenState == RobotSystems.collectSpecimenStates.WAITING) {
                    robotSystems.collectSpecimenState = RobotSystems.collectSpecimenStates.MAX_RETRACT_FUNNY;
                } else {
                    intakeSubsystem.claw.toggle();
                }
            }

            //rotations(both of them)
            if(robotSystems.transferState == RobotSystems.TransferStates.IDLE){
                intakeSubsystem.rotation.handleRotation(gamepad2);
            }


            if(gamepad2.dpad_down)  {
                extendo.goToGround();
                intakeSubsystem.goToWall();
            }
            if(gamepad2.dpad_up && lift.isDown()) {
                extendo.goToPos(PositionsExtendo.maxLegal);
                intakeSubsystem.goDown();
                robotSystems.collectSpecimenState = RobotSystems.collectSpecimenStates.IDLE;
            }

            //lift
            if(gamepad2.b && !robotSystems.isTransfering()) {
                lift.goToHighChamber();
                outtakeSubsystem.goToSpecimenScore();
            }
            if(gamepad2.y && !robotSystems.isTransfering()) {
                lift.goToHighBasket();
                outtakeSubsystem.goToSampleScore();
            }
            if(gamepad2.a) {
                lift.goToGround();
                outtakeSubsystem.goToTransfer();
//                if(intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.TRANSFER) intakeSubsystem.goToReady(robotSystems.transferingSample);
            }

            if(gamepad2.right_bumper) outtakeSubsystem.claw.open();


            stickyGamepad2.update();
            stickyGamepad1.update();
            robotSystems.update();

            //reset lift
            if(gamepad1.dpad_down) {
                while (gamepad1.dpad_down) {
                    lift.setPower(-0.5);
                }
                lift.setPower(0);
                lift.reset();
                gamepad1.rumble(100);
            }

            if(gamepad1.dpad_up){
                while(gamepad1.dpad_up){
                    extendo.setPower(-0.35);
                }
                extendo.setPower(0);
                extendo.reset();
                gamepad1.rumble(100);
            }


            if(stickyGamepad2.right_bumper){
                intakeSubsystem.claw.open();
            }

            //telemetry:
            telemetry.addData("[STATUS]", "Main Teleop is running.");
            telemetry.addData("Match Time", matchTimer.seconds());
            telemetry.addData("Lift", lift.currentPos);
            telemetry.addData("lift is at POS", lift.isAtPosition());
            telemetry.addData("Extendo", extendo.currentPos);
            telemetry.addData("Intake", intakeSubsystem.intakeState);
            telemetry.addData("intakeTimer", robotSystems.timerCollect.milliseconds());
            telemetry.addData("extendoPID", extendo.pidEnabled);
//            telemetry.addData("outtakeRot", robotSystems.outtakeSubsystem.joint.getRot());
            telemetry.addData("intakeRot", intakeSubsystem.rotation.rotLevel);

            double voltage = batteryVoltageSensor.getVoltage();
            telemetry.addData("Battery Voltage", voltage);

            // telemetry.addData("BreakBeam", intakeSubsystem.breakBeam.hasElement());

            telemetry.update();
        }
    }

}

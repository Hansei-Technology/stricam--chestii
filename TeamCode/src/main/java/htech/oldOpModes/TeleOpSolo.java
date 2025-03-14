//package htech;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import htech.classes.StickyGamepad;
//import htech.subsystem.ChassisMovement;
//import htech.subsystem.ExtendoSystem;
//import htech.subsystem.IntakeSubsystem;
//import htech.subsystem.LiftSystem;
//import htech.subsystem.OuttakeSubsystem;
//import htech.subsystem.RobotSystems;
//
//@TeleOp(name = "[TELEOP] SOLO", group = "HTech")
//public class TeleOpSolo extends LinearOpMode {
//
//    ChassisMovement chassisMovement;
//    IntakeSubsystem intakeSubsystem;
//    OuttakeSubsystem outtakeSubsystem;
//    LiftSystem lift;
//    ExtendoSystem extendo;
//    RobotSystems robotSystems;
//    ElapsedTime timer;
//
//    boolean sample = true;
//    boolean extended = false;
//
//    public enum RightBumperStatesSample {
//        EXTEND,
//        COLLECT,
//    }
//    RightBumperStatesSample rightBumperStateSample = RightBumperStatesSample.EXTEND;
//
//
//    public enum LeftBumperStatesSample {
//        GO_TO_BASKET,
//        CLAW_INTAKE,
//        CLAW_OUTTAKE,
//    }
//    LeftBumperStatesSample leftBumperStateSample = LeftBumperStatesSample.CLAW_INTAKE;
//
//
//    public enum RightBumperStatesSpecimen{
//        TOGGLE_EXTEND,
//        COLLECT,
//    }
//    RightBumperStatesSpecimen rightBumperStateSpecimen = RightBumperStatesSpecimen.TOGGLE_EXTEND;
//
//    public enum LeftBumperStatesSpecimen{
//        CLAW_COLLECT,
//        CLAW_SCORE,
//        SCORE_SPECIMEN,
//    }
//    LeftBumperStatesSpecimen leftBumperStateSpecimen = LeftBumperStatesSpecimen.CLAW_COLLECT;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        chassisMovement = new ChassisMovement(hardwareMap);
//        intakeSubsystem = new IntakeSubsystem(hardwareMap);
//        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
//        lift = new LiftSystem(hardwareMap);
//        extendo = new ExtendoSystem(hardwareMap);
//        robotSystems = new RobotSystems(extendo, lift, intakeSubsystem, outtakeSubsystem);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        timer = new ElapsedTime();
//
//        StickyGamepad stickyGamepad2 = new StickyGamepad(gamepad2, this);
//        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1, this);
//
//        waitForStart();
//
//        intakeSubsystem.init();
//        outtakeSubsystem.init();
//
//        robotSystems.fastCollect = true;
//
//        while (opModeIsActive()) {
//
//            if(robotSystems.transferState == RobotSystems.TransferStates.IDLE){
//                intakeSubsystem.rotation.handleRotation(gamepad1);
//            }
//
//            stickyGamepad1.update();
//            stickyGamepad2.update();
//            robotSystems.update();
//
//            chassisMovement.updateMovementSlowRotation(gamepad1);
//
//            if(gamepad1.touchpad){
//                sample = !sample;
//                robotSystems.fastCollect = !robotSystems.fastCollect;
//                gamepad1.rumble(150);
//            }
//
//            if(sample){
//
//                switch (rightBumperStateSample){
//
//                    case EXTEND:
//                        if(gamepad1.right_bumper){
//                            extendo.goToMax();
//                            lift.goToGround();
//                            outtakeSubsystem.goToTransfer();
//                            rightBumperStateSample = RightBumperStatesSample.COLLECT;
//                        }
//                        break;
//
//                    case COLLECT:
//                        if(stickyGamepad1.right_bumper){
//                            intakeSubsystem.collect(false);
//                        }
//                        if(intakeSubsystem.hasElement()){
//                            rightBumperStateSample = RightBumperStatesSample.EXTEND;
//                        }
//                        break;
//
//
//                }
//
//                switch (leftBumperStateSample){
//
//                    case GO_TO_BASKET:
//                        if(stickyGamepad1.left_bumper){
//                            lift.goToHighBasket();
//                            outtakeSubsystem.goToSampleScore();
//                            leftBumperStateSample = LeftBumperStatesSample.CLAW_OUTTAKE;
//                        }
//                        break;
//
//                    case CLAW_INTAKE:
//                        if(stickyGamepad1.left_bumper){
//                            intakeSubsystem.claw.toggle();
//                        }
//                        if(intakeSubsystem.hasElement()){
//                            leftBumperStateSample = LeftBumperStatesSample.GO_TO_BASKET;
//
//                        }
//                        break;
//
//                    case CLAW_OUTTAKE:
//                        if(stickyGamepad1.left_bumper){
//                            outtakeSubsystem.claw.open();
//                            leftBumperStateSample = LeftBumperStatesSample.CLAW_INTAKE;
//                        }
//                        break;
//                }
//            }
//            else{
//
//                switch (rightBumperStateSpecimen){
//
//                    case TOGGLE_EXTEND:
//                        if(gamepad1.right_bumper){
//                            extendo.goToMax();
//                            rightBumperStateSpecimen = RightBumperStatesSpecimen.COLLECT;
//                        }
//                        break;
//
//                    case COLLECT:
//                        if(stickyGamepad1.right_bumper){
//                            intakeSubsystem.collect(false);
//                        }
//                        if(intakeSubsystem.hasElement() && !intakeSubsystem.claw.isOpen){
//                            rightBumperStateSpecimen = RightBumperStatesSpecimen.TOGGLE_EXTEND;
//                            extendo.goToGround();
//                        }
//                        break;
//                }
//
//                switch (leftBumperStateSpecimen){
//
//                    case CLAW_COLLECT:
//                        if(stickyGamepad1.left_bumper){
//                            intakeSubsystem.claw.toggle();
//                        }
//                        if(gamepad1.left_stick_button){
//                            leftBumperStateSpecimen = LeftBumperStatesSpecimen.CLAW_SCORE;
//                        }
//                        break;
//
//                    case CLAW_SCORE:
//                        if(stickyGamepad1.left_bumper){
//                            intakeSubsystem.claw.toggle();
//                        }
//                        if(intakeSubsystem.hasElement() && !intakeSubsystem.claw.isOpen){
//                            timer.reset();
//                            robotSystems.startTransfer(false);
//                            if(timer.milliseconds() > 300 && robotSystems.transferState == RobotSystems.TransferStates.IDLE){
//                                lift.goToHighChamber();
//                                outtakeSubsystem.goToSpecimenVertical();
//                            }
//                        }
//                        if(gamepad1.left_stick_button){
//                            leftBumperStateSpecimen = LeftBumperStatesSpecimen.CLAW_COLLECT;
//                        }
//                        break;
//
//                    case SCORE_SPECIMEN:
//                        if(stickyGamepad1.left_bumper){
//                            robotSystems.placeVertical();
//                            leftBumperStateSpecimen = LeftBumperStatesSpecimen.CLAW_SCORE;
//                        }
//                        break;
//                }
//
//            }
//
//
//            //reset lift
//            if(gamepad1.dpad_down) {
//                while (gamepad1.dpad_down) {
//                    lift.setPower(-0.5);
//                }
//                lift.setPower(0);
//                lift.reset();
//                gamepad1.rumble(100);
//            }
//
//            if(gamepad1.dpad_up){
//                while(gamepad1.dpad_up){
//                    extendo.setPower(-0.35);
//                }
//                extendo.setPower(0);
//                extendo.reset();
//                gamepad1.rumble(100);
//            }
//
//        }
//    }
//}

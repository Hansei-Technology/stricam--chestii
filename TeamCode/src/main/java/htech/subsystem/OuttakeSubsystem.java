package htech.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import htech.mechanism.outtake.OuttakeBar;
import htech.mechanism.outtake.OuttakeClaw;
import htech.mechanism.outtake.OuttakeFunny;

@Config
public class OuttakeSubsystem {
    public final OuttakeClaw claw;
    public final OuttakeBar bar;
    public final OuttakeFunny funny;
    //public final LimitSwitch limitSwitch;

    public enum outtakeStates {
        SPECIMEN,
        BASKET,
        TRANSFER
    }
    public outtakeStates CS = outtakeStates.TRANSFER;


    public OuttakeSubsystem(HardwareMap hardwareMap) {
        // MECHANISM //
        claw = new OuttakeClaw(hardwareMap);
        bar = new OuttakeBar(hardwareMap);
        funny = new OuttakeFunny(hardwareMap);
    }

    public void init() {
        bar.goToTransfer();
        claw.close();
        funny.retract();
    }

    public void goToTransferSample() {
        bar.goToSample();
        funny.retract();
        claw.open();
    }

    public void goToTransfer() {
        bar.goToTransfer();
        funny.goToTransfer();
        claw.open();
    }

    public void goToSampleScore() {
        bar.goToSample();
        funny.maxRetract();
    }

    public void goToSpecimenScore() {
        bar.goToSpecimenScore();
    }

    public void goToSpecimenPrescore(){
        bar.goToSpecimenPrescore();
        funny.extend();
    }

    public void goToAfterTransfer() {
        bar.goToAfterTransfer();
        funny.maxRetract();
    }

    public void goToPreCollectSpecimen() { //
        bar.goToSpecimenCollect();
        funny.retract();
        claw.open();
    }

    public void goToCollectSpecimen(){ //gen cand se retrage glisiera ca sa prinda specmenu de pe perete
        bar.goToSpecimenCollect();
        funny.retract();
    }

    public void retractFunny() {
        funny.retract();
    }
}

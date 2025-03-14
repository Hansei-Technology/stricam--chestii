package htech.classes;

import com.qualcomm.robotcore.hardware.Servo;

public class DualServoController {

    private final CoolServo servo1, servo2;
    private final AsymmetricMotionProfile motionProfile;
    private double targetPosition;

    public DualServoController(Servo servo1, Servo servo2) {
        this(servo1, servo2, false, false, 100, 50, 10, 0.5);
    }

    public DualServoController(Servo servo1, Servo servo2, boolean reversed1, boolean reversed2,
                               double maxVelocity, double acceleration, double deceleration, double initialPosition) {
        this.servo1 = new CoolServo(servo1, reversed1, maxVelocity, acceleration, deceleration, initialPosition);
        this.servo2 = new CoolServo(servo2, reversed2, maxVelocity, acceleration, deceleration, initialPosition);
        this.motionProfile = new AsymmetricMotionProfile(maxVelocity, acceleration, deceleration);
        motionProfile.setMotion(initialPosition, initialPosition, 0);
        this.targetPosition = initialPosition;
    }

    public void setTargetPosition(double position) {
        if (position == targetPosition) return; // Dacă poziția e deja setată, nu face nimic
        targetPosition = position;
        motionProfile.setMotion(motionProfile.getPosition(), position, motionProfile.getVelocity());
    }

    public void update() {
        motionProfile.update();
        double profilePosition = motionProfile.getPosition();

        // Setăm poziția fiecărui servo în funcție de profil
        servo1.setPosition(profilePosition);
        servo2.setPosition(profilePosition);

        // Actualizăm starea lor
        servo1.update();
        servo2.update();
    }

    public boolean isMotionComplete() {
        return motionProfile.getTimeToMotionEnd() <= 0; // Verificăm dacă mișcarea s-a terminat
    }

    public void forceUpdate() {
        motionProfile.update();
        servo1.forceUpdate();
        servo2.forceUpdate();
    }

    public void telemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Profile Position", motionProfile.getPosition());
        telemetry.addData("Motion Complete", isMotionComplete());
        motionProfile.telemetry(telemetry);
    }
}

package htech.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import htech.classes.HanseiTelemetry;
import htech.classes.StickyGamepad;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
@TeleOp(name = "[UTIL] Multiple Servo Tester", group = "HTech")
public class MultipleServoTester extends LinearOpMode {
    public static int length = 2;
    public static String[] servos = new String[10];
    public static double[] positions = new double[10];
    private HanseiTelemetry htelemetry;

    public static boolean configure = true;
    public static int pos_leftServo = 0;
    public static int pos_rightServo = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry mtelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1, this);
        htelemetry = new HanseiTelemetry(mtelemetry);
        htelemetry.clear();

        if (length <= 0 || length > 10) {
            htelemetry.error("Length must be between 1 and 10.");
            htelemetry.info("This tester will now stop.");
            htelemetry.update();
            stop();
            return;
        }

        if (configure) {
            htelemetry.status("CONFIG");
            htelemetry.info("Configuration mode will change the length of the arrays, while keeping data.");
            htelemetry.info("After this, it will stop, and you'll be able to freely configure the tester.");
            htelemetry.update();

            waitForStart();
            if (!opModeIsActive()) {
                htelemetry.clear();
                return;
            }

            resetData();
            htelemetry.success("Data has been updated successfully!");
            htelemetry.update();
            stop();
            return;
        }

        htelemetry.status("INIT");
        htelemetry.info("All checks passed. Start the OpMode to register servos.");
        htelemetry.update();

        waitForStart();
        Servo[] servoInstances = new Servo[servos.length];

        for (int i = 0; i < servos.length; i++) {
            htelemetry.status("INIT");
            htelemetry.status("Initializing Servos...");

            try {
                servoInstances[i] = hardwareMap.get(Servo.class, servos[i]);

                for (int j = 0; j <= i; j++) {
                    htelemetry.success("Initialized servo '" + servos[j] + "'.");
                }
                htelemetry.update();
            } catch(Exception e) {
                htelemetry.error("Servo with value '" + servos[i] + "' does not exist!");
                htelemetry.error("Program will now exit");
                htelemetry.update();
                stop();
                return;
            }
        }

        htelemetry.clear();

        for (int i = 0; i < servos.length; i++) {
            positions[i] = servoInstances[i].getPosition();
        }

        FtcDashboard.getInstance().updateConfig();

        while (opModeIsActive()) {
            selectionLogic(stickyGamepad);

            htelemetry.info("Selected Left Servo: " + pos_leftServo + " | " + servos[pos_leftServo]);
            htelemetry.info("Position: " + positions[pos_leftServo]);
            htelemetry.info("Selected Right Servo: " + pos_rightServo + " | " + servos[pos_rightServo]);
            htelemetry.info("Position: " + positions[pos_rightServo]);

            positionsLogic(stickyGamepad);
            updatePositions(servoInstances);

            htelemetry.status("RUNNING");
            htelemetry.update();
            FtcDashboard.getInstance().updateConfig();
        }
    }

    /**
     * Resets the servo and position arrays. Not only that, this also keeps the previous data.
     */
    private void resetData() {
        if (servos != null && positions != null) {
            String[] tempServos = servos.clone();
            double[] tempPositions = positions.clone();

            servos = new String[length];
            positions = new double[length];
            Arrays.fill(servos, "");

            for (int i = 0; i < tempServos.length && i < length; i++) {
                if (tempServos[i] != null) {
                    servos[i] = tempServos[i];
                }
                positions[i] = tempPositions[i];
            }
        } else {
            servos = new String[length];
            positions = new double[length];
            Arrays.fill(servos, "");
        }

        FtcDashboard.getInstance().updateConfig();
    }

    /**
     * Handles the selection logic for the servos.
     * @param gamepad A sticky gamepad to be used for the logic.
     */
    private void selectionLogic(StickyGamepad gamepad) {
        if (gamepad.left_bumper) {
            pos_leftServo++;
            if (pos_leftServo >= length) pos_leftServo = 0;
        }

        if (gamepad.right_bumper) {
            pos_rightServo++;
            if (pos_rightServo >= length) pos_rightServo = 0;
        }
    }

    /**
     * Handles changing the positions for the servos.
     * @param gamepad A sticky gamepad to be used for the logic.
     */
    private void positionsLogic(StickyGamepad gamepad) {
        if (gamepad.dpad_up && positions[pos_leftServo] < 1)
            positions[pos_leftServo] += 0.01;
        else if (gamepad.dpad_down && positions[pos_leftServo] > 0)
            positions[pos_leftServo] -= 0.01;

        if (gamepad.y && positions[pos_rightServo] < 1)
            positions[pos_rightServo] += 0.01;
        else if (gamepad.a && positions[pos_rightServo] > 0)
            positions[pos_rightServo] -= 0.01;
    }

    /**
     * Updates all the servo positions.
     * @param servos All the servo instances.
     */
    private void updatePositions(Servo[] servos) {
        for (int i = 0; i < servos.length; i++) {
            servos[i].setPosition(positions[i]);
        }
    }
}

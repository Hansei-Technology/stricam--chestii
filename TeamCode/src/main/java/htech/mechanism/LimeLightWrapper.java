package htech.mechanism;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class LimeLightWrapper {
    LLResult result;
    public double[] pythonOutput;
    public boolean valid;
    Limelight3A limelight;
    public static int pipeLine = 0;
    public double X, Y, HEADING;

    // Static calibration values
    public static int py50 = 470;
    public static int py70 = 460;
    public static int py90 = 450;
    public static int py110 = 440;
    public static int py130 = 430;
    public static int py150 = 425;
    public static int py170 = 365;
    public static int py200 = 335;
    public static int py210 = 310;
    public static int py220 = 300;
    public static int py240 = 290;
    public static int py260 = 230;
    public static int py280 = 220;
    public static int py310 = 200;
    public static int py320 = 170;
    public static int py340 = 150;
    public static int py360 = 123;
    public static int py380 = 110;
    public static int py400 = 65;
    public static int py420 = 40;

    public int[][] referenceMatrix;

    public static double px50 = 0.05;
    public static double px70 = 0.048;
    public static double px90 = 0.046;
    public static double px110 = 0.043;
    public static double px130 = 0.042;
    public static double px150 = 0.041;
    public static double px170 = 0.038;
    public static double px200 = 0.037;
    public static double px210 = 0.036;
    public static double px220 = 0.035;
    public static double px240 = 0.034;
    public static double px260 = 0.031;
    public static double px280 = 0.03;
    public static double px300 = 0.03;
    public static double px320 = 0.029;
    public static double px340 = 0.028;
    public static double px360 = 0.025;
    public static double px380 = 0.02;
    public static double px400 = 0.02;
    public static double px420 = 0.02;

    public double[][] XMultiplierMatrix;

    public LimeLightWrapper(HardwareMap map) {
        limelight = map.get(Limelight3A.class, "limelight");
        updateMatrices(); // Ensure matrices are initialized
    }

    public void start() {
        limelight.start();
    }

    public void update() {
        updateMatrices(); // Ensure the matrices use the latest values

        result = limelight.getLatestResult();
        pythonOutput = result.getPythonOutput();

        valid = (pythonOutput[0] == 1);

        if (valid) {
            X = (pythonOutput[1] - 318) * getMultiplyerValueforX(pythonOutput[2]);
            Y = getIntervalValueforY(pythonOutput[2]);
            HEADING = pythonOutput[3] <= 90? pythonOutput[3] : 180 - pythonOutput[3];
        }
    }

    public void updateMatrices() {
        referenceMatrix = new int[][] {
                {50, py50}, {70, py70}, {90, py90}, {110, py110}, {130, py130},
                {150, py150}, {170, py170}, {200, py200}, {210, py210}, {220, py220},
                {240, py240}, {260, py260}, {280, py280}, {310, py310}, {320, py320},
                {340, py340}, {360, py360}, {380, py380}, {400, py400}, {420, py420}
        };

        XMultiplierMatrix = new double[][] {
                {50, px50}, {70, px70}, {90, px90}, {110, px110}, {130, px130},
                {150, px150}, {170, px170}, {200, px200}, {210, px210}, {220, px220},
                {240, px240}, {260, px260}, {280, px280}, {300, px300}, {320, px320},
                {340, px340}, {360, px360}, {380, px380}, {400, px400}, {420, px420}
        };
    }

    public int getIntervalValueforY(double pixelDistance) {
        for (int i = 0; i < referenceMatrix.length - 1; i++) {
            int px1 = referenceMatrix[i][0];
            int px2 = referenceMatrix[i + 1][0];
            int realValue = referenceMatrix[i][1];

            if (pixelDistance >= px1 && pixelDistance < px2) {
                return realValue;
            }
        }
        return referenceMatrix[referenceMatrix.length - 1][1];
    }

    public double getMultiplyerValueforX(double pixelDistance) {
        for (int i = 0; i < XMultiplierMatrix.length - 1; i++) {
            double px1 = XMultiplierMatrix[i][0];
            double px2 = XMultiplierMatrix[i + 1][0];
            double realValue = XMultiplierMatrix[i][1];

            if (pixelDistance >= px1 && pixelDistance < px2) {
                return realValue;
            }
        }
        return 0;
    }
}

/* Copyright (c) 2017-2020 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package htech.mechanism.intake;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import htech.config.PositionsIntake;
import htech.config.Sensors;

public class ColorSensor {

    NormalizedColorSensor colorSensor;
    public enum Colors {
        RED, GREEN, BLUE;
    }
    public Colors biggestColor;
    public float max;
    float gain;

    public ColorSensor(HardwareMap map) {
        colorSensor = map.get(NormalizedColorSensor.class, Sensors.colorSensor);
        gain = PositionsIntake.colorSensorGain;
        colorSensor.setGain(gain);
    }

    public void getColors() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        max = Math.max(Math.max(colors.red, colors.green), colors.blue);
        if(max == colors.red) {
            biggestColor = Colors.RED;
        } else if(max == colors.green) {
            biggestColor = Colors.GREEN;
        } else if(max == colors.blue) {
            biggestColor = Colors.BLUE;
        }
    }

    public double getDistance() {
        return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
    }

    public boolean hasElement() {
        getColors();
        if(biggestColor == Colors.RED && max > PositionsIntake.thresholdRed) {
            return true;
        } else if(biggestColor == Colors.GREEN && max > PositionsIntake.thresholdGreen) {
            return true;
        } else if(biggestColor == Colors.BLUE && max > PositionsIntake.thresholdBlue) {
            return true;
        } else
            return false;
    }
}

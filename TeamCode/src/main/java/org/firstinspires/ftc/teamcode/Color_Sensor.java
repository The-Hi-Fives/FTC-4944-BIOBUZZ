package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class Color_Sensor extends OpMode {
    Color_Sensor rColor;
    Color_Sensor lColor;
    Servo adjuster;
    NormalizedRGBA colors;

    @Override
    public void init() {
        rColor = hardwareMap.get(Color_Sensor.class, "rColor");
        lColor = hardwareMap.get(Color_Sensor.class, "lColor");
        adjuster = hardwareMap.get(Servo.class, "adjuster");


        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f", colors.green);
        telemetry.addData("Blue", "%.3f", colors.blue);
        telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));
        telemetry.addData("Saturation", "%.3f", JavaUtil.colorToSaturation(colors.toColor()));
        telemetry.addData("Value", "%.3f", JavaUtil.colorToValue(colors.toColor()));
    }

    @Override
    public void loop() {
        if (lColor.colors.red> 100){
            adjuster.setPosition(1);
            telemetry.addData("shooting value,", "shoot");
        } else if (lColor.colors.blue>100) {
            adjuster.setPosition(1);
            telemetry.addData("Opponents Ball,", "outtake");
        }else if (lColor.colors.green>100){
            adjuster.setPosition(0.5);
            telemetry.addData("Pollen", "shoot");
        }else{
            adjuster.setPosition(0);
            telemetry.addData("No Balls,", "go intake");
        }



    }
}

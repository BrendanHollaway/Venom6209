package com.qualcomm.ftcrobotcontroller.opmodes.Unused_Files;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.hardware.ModernRoboticsNxtUltrasonicSensor;
//import com.qualcomm.hardware.ModernRoboticsOpticalDistanceSensor;
//import com.qualcomm.robotcore.hardware.UltrasonicSensor;
//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
/**
 * Created by viperbots on 10/15/2015.
 */
public class ColorSensorTest extends OpMode{
    ColorSensor color;

    @Override
    public void init(){
        color = hardwareMap.colorSensor.get("color");
    }
    public void loop() {
        telemetry.addData("blue: ", String.format("%.2f", color.blue()));
        telemetry.addData("red: ", String.format("%.2f", color.red()));
    }
}
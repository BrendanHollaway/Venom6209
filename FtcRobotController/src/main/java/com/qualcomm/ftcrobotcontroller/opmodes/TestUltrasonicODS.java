package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.hardware.ModernRoboticsNxtUltrasonicSensor;
//import com.qualcomm.hardware.ModernRoboticsOpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
/**
 * Created by viperbots on 10/15/2015.
 */
public class TestUltrasonicODS extends OpMode{
    OpticalDistanceSensor optic;
    UltrasonicSensor us;

    @Override
    public void init(){
        optic = hardwareMap.opticalDistanceSensor.get("optical");
        us = hardwareMap.ultrasonicSensor.get("ultra");
    }
    public void loop() {
        telemetry.addData("optical: ", String.format("%.2f", optic.getLightDetected()));
        telemetry.addData("ultrasonic: ", String.format("%.2f", us.getUltrasonicLevel()));
    }
}

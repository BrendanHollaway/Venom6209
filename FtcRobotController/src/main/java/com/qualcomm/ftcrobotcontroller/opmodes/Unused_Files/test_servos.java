package com.qualcomm.ftcrobotcontroller.opmodes.Unused_Files;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by viperbots on 1/25/2016.
 */
public class test_servos extends OpMode {
    Servo test;
    public void init()
    {
        test = hardwareMap.servo.get("servoTest");
    }
    public void loop()
    {
        if(System.currentTimeMillis() % 2000 > 1000)
            test.setPosition(0.5);
        else
            test.setPosition(0);
    }
}

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by viperbots on 12/15/2015.
 */
public class ServoTesssst extends OpMode{
    Servo servo;
    int count = 0;
    public void init() {
        servo = hardwareMap.servo.get("servo1"); //dont judge
    }
    public void loop()
    {
        if(servo.getPosition() == 0 && count++ < 10)
            servo.setPosition(0);
        else if(count++ < 20)
            servo.setPosition(1);
        else {
            count = 0;
            servo.setPosition(0);
        }
    }

}

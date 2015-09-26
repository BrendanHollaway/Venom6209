package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Created by viperbots on 9/17/2015.
 */



public class ArmThread implements Runnable{
    public volatile float armPower;
    public void run()
    {
        while(!Thread.currentThread().isInterrupted())
        {
          //  armPower = gamepad2.right_stick_y;
          //  armVertical.setPower(armPower);
        }
    }
}
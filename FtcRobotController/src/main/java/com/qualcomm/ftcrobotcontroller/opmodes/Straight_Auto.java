package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by viperbots on 12/4/2015.
 */
public class Straight_Auto extends LinearOpModeCV {
    AutonomousSegments auto;
    @Override
    public void runOpMode() throws InterruptedException {
        super.map();
        waitForStart();
        auto = new AutonomousSegments(motorFL, motorBL, motorFR, motorBR, IMU, telemetry, this);
        int encoder = motorFR.getCurrentPosition();
        //auto.Close_Blue_Buttons_CV();
        auto.PID_move(10000 + encoder, 0, 1, false);
        /*motorFR.setPower(.8);
        motorFL.setPower(-.2);
        motorBL.setPower(-.2);
        motorBR.setPower(.8);
        long time = System.nanoTime() + 10 * (long) Math.pow(10, 9);
        while(System.nanoTime() < time)
            waitOneFullHardwareCycle();
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);*/
    }
}


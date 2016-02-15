package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by viperbots on 12/4/2015.
 */
public class Straight_Auto extends LinearOpMode2 {
    @Override
    public void runOpMode() throws InterruptedException {
        super.map();
        waitForStart();
        motorFR.setPower(1);
        motorFL.setPower(1);
        motorBL.setPower(1);
        motorBR.setPower(1);
        wait(4000);
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
}


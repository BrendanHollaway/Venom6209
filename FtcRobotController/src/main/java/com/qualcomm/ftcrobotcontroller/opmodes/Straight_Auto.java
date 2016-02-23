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
        auto.Regionals_Straight();
    }
}


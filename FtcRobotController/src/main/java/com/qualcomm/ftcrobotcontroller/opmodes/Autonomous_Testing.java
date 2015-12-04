package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by viperbots on 9/29/2015.
 */
public class Autonomous_Testing extends LinearOpMode {
    DcMotor motorBL;
    DcMotor motorBR;
    AutonomousSegments auto;
    public void runOpMode() {
        auto.move_pos(0, 100 ,1);
    }
}

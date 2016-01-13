package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.opmodes.AutonomousSegments;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

public class autotest extends LinearOpMode2 {
    AutonomousSegments auto;

    public void runOpMode() throws InterruptedException {

        super.map();

        motorFL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        auto = new AutonomousSegments(motorFL, motorBL, motorBR, motorFR, IMU, telemetry);
        /*long time = (long) (System.nanoTime() + Math.pow(10, 9.5));
        while(System.nanoTime() < time)
            telemetry.addData("gyro yaw; ", gyroTest());*/
        waitForStart();
        telemetry.addData("autonomous: ", "Started");
        auto.Close_Blue_Buttons();
        auto.Climbers();
        //auto.Buttons();
        //auto.BlueButtons_RedRamp();
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        stop();
    }
    public double gyroTest() {
        IMU.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return yawAngle[0];
    }
}
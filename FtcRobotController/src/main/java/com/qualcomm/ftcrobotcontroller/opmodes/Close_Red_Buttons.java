package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;
/**
 * Created by viperbots on 10/5/2015.
 */
public class Close_Red_Buttons extends LinearOpMode {

    DcMotor motorFL;
    DcMotor motorBL;
    DcMotor motorFR;
    DcMotor motorBR;



    public void ssleep(long ms) throws InterruptedException
    {
        try {
            sleep(ms);
        }
        catch (Exception E){}
    }

    public void move(double power, int ms) throws InterruptedException
    {
        motorFL.setTargetPosition(0);// .setPower(power);
        motorBL.setPower(power);
        motorFR.setPower(power);
        motorBR.setPower(power);
        halt();
    }
    public void left(double power, int ms) throws InterruptedException
    {
        motorFL.setPower(power);
        motorBL.setPower(power);
        motorFR.setPower(0);
        motorBR.setPower(0);
        halt();
    }
    public void right(double power, int ms) throws InterruptedException
    {
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(power);
        motorBR.setPower(power);
        halt();
    }
    public void halt()
    {
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }

    public void runOpMode() throws InterruptedException {

        motorFL = hardwareMap.dcMotor.get("motor_1");
        motorBL = hardwareMap.dcMotor.get("motor_2");
        motorFR = hardwareMap.dcMotor.get("motor_3");
        motorBR = hardwareMap.dcMotor.get("motor_4");
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);


    }
}
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;
/**
 * Created by viperbots on 10/5/2015.
 */
public class Far_Red_Buttons extends LinearOpMode {

    DcMotor motorFL;
    DcMotor motorBL;
    DcMotor motorFR;
    DcMotor motorBR;

    double cm_rotation = 1.5*Math.PI*2.54;
    double square = 60/cm_rotation;



    public void ssleep(long ms) throws InterruptedException
    {
        try {
            sleep(ms);
        }
        catch (Exception E){}
    }

    public void move(double position, double speed) throws InterruptedException
    {
        while(Math.abs(motorFL.getCurrentPosition()) < position ) {
            motorFL.setPower(Math.signum(position) * Math.abs(speed));
            motorBL.setPower(Math.signum(position) * Math.abs(speed));
            motorFR.setPower(Math.signum(position) * Math.abs(speed));
            motorBR.setPower(Math.signum(position) * Math.abs(speed));
        }
        halt();
    }
    public void turn(double deg, double speed) throws InterruptedException //pos deg is turn clockwise (Deg measured after transformation)
    {
        deg %= 360.0;
        if(deg > 180)
        {
            deg -= 360;
        }
        else if(deg < -180)
        {
            deg += 360;
        }
        double getGyro = 0.0;
        while(Math.abs(getGyro) < Math.abs(deg))
        {
            motorFL.setPower(Math.signum(deg) * Math.abs(speed));
            motorBL.setPower(Math.signum(deg) * Math.abs(speed));
            motorFR.setPower(-Math.signum(deg) * Math.abs(speed));
            motorBR.setPower(-Math.signum(deg) * Math.abs(speed));
        }
        halt();
    }
    /*public void left(double position, double speed) throws InterruptedException
    {
        while(Math.abs(motorFL.getCurrentPosition()) < position ) {
            motorFL.setPower(Math.signum(position) * Math.abs(speed));
            motorBL.setPower(Math.signum(position) * Math.abs(speed));
            motorFR.setPower(0);
            motorBR.setPower(0);
        }
        halt();
    }
    public void right(double position, double speed) throws InterruptedException
    {
        while(Math.abs(motorFR.getCurrentPosition()) < position ) {
            motorFL.setPower(0);
            motorBL.setPower(0);
            motorFR.setPower(Math.signum(position) * Math.abs(speed));
            motorBR.setPower(Math.signum(position) * Math.abs(speed));
        }
        halt();
    }*/
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
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

        move(-square, 1);
        turn(45,1);
        move(-Math.sqrt(18) * square, 1);
        turn(45,1);
        move(-square, 1);

    }
}

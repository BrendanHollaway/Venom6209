package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
/**
 * Created by viperbots on 10/5/2015.
 */
public class AutonomousSegments extends LinearOpMode {

    DcMotor motorFL;
    DcMotor motorBL;
    DcMotor motorFR;
    DcMotor motorBR;
    UltrasonicSensor ultra;
    UltrasonicSensor frontUltra;

    double cm_rotation = 1.5*Math.PI*2.54;
    double square = 60/cm_rotation;
    double inches = 1.5*Math.PI;


    public AutonomousSegments()
    {
        this.motorFL = null;
        this.motorBL = null;
        this.motorFR = null;
        this.motorBR = null;
    }


    public AutonomousSegments(DcMotor motorFL, DcMotor motorBL, DcMotor motorFR, DcMotor motorBR)
    {
        this.motorFL = motorFL;
        this.motorBL = motorBL;
        this.motorFR = motorFR;
        this.motorBR = motorBR;
    }

    public void setupMotors () {
        motorFL = hardwareMap.dcMotor.get("motor_1");
        motorBL = hardwareMap.dcMotor.get("motor_2");
        motorFR = hardwareMap.dcMotor.get("motor_3");
        motorBR = hardwareMap.dcMotor.get("motor_4");
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }

    public void ssleep(long ms) throws InterruptedException
    {
        try {
            sleep(ms);
        }
        catch (Exception E){}
    }

    public void move(double position, double speed) throws InterruptedException
    {
        int currentPosition = motorFL.getCurrentPosition();
        while(Math.abs(motorFL.getCurrentPosition()) < position + currentPosition ) {
            motorFL.setPower(Math.signum(position) * Math.abs(speed));
            motorBL.setPower(Math.signum(position) * Math.abs(speed));
            motorFR.setPower(Math.signum(position) * Math.abs(speed));
            motorBR.setPower(Math.signum(position) * Math.abs(speed));
        }
        halt();
    }
    public void moveCheck(double position, double speed) throws InterruptedException
    {
        int currentPosition = motorFL.getCurrentPosition();
        while(Math.abs(motorFL.getCurrentPosition()) < position + currentPosition ) {
            checkObject();
            motorFL.setPower(Math.signum(position) * Math.abs(speed));
            motorBL.setPower(Math.signum(position) * Math.abs(speed));
            motorFR.setPower(Math.signum(position) * Math.abs(speed));
            motorBR.setPower(Math.signum(position) * Math.abs(speed));
        }
        halt();
    }
    public void checkObject() throws InterruptedException
    {
        if (frontUltra.getUltrasonicLevel() < 10)
        {
            halt();
            ssleep(3000);
            /*if (frontUltra.getUltrasonicLevel() < 10)
            {
                turn(90, 1);
                move(30 * inches, 1);
                turn(-90, 1);
                move(48 * inches, 1);
                turn(-90, 1);
                move(30 * inches, 1);
                turn(90, 1);
            }*/
        }
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
    public void halt() {
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }
    void SOS(double acc_y, double acc_z)
    {
        if (acc_y < -8.88 && acc_z > -4.14)
        {
            motorBL.setPower(-1);
            motorBR.setPower(-1);
            motorFL.setPower(-1);
            motorFR.setPower(-1);
            try {
                wait(500);
            }
            catch (Exception E){}
            motorBL.setPower(0);
            motorBR.setPower(0);
            motorFL.setPower(0);
            motorFR.setPower(0);
        }
    }

//RED INITIAL SEGMENTS

    public void Close_Red_Buttons() throws InterruptedException {
        move(-2 * square, 1);
        turn(45, 1);
        move(-1.5 * (Math.sqrt(2)) * square, 1);
        turn(45, 1);
        move(-0.5 * square,1);
    }
    public void Far_Red_Buttons() throws InterruptedException {
        move(-square, 1);
        turn(45, 1);
        move(-2 * (Math.sqrt(2)) * square, 1);
        turn(45, 1);
        move(-2 * square, 1);
    }
    public void Close_Red_RedRamp() throws InterruptedException {
        move(square, 1);
        turn(-45, 1);
        move(-Math.sqrt(2) * square, 1);
        turn(-90, 1);
    }
    public void Far_Red_RedRamp() throws InterruptedException {
        move(2 * square, 1);
        turn(-90, 1);
        move(3 * square, 1);
        turn(45, 1);
        move(.5 * Math.sqrt(2) * square, 1);
    }
    public void Close_Red_BlueRamp() throws InterruptedException {
        move(3 * square, 1);
        turn(45, 1);
        move(2 * Math.sqrt(2) * square, 1);
    }
    public void Far_Red_BlueRamp() throws InterruptedException {
        move(3.5 * square, 1);
        turn(45, 1);
    }

//BLUE INITIAL SEGMENTS

    public void Close_Blue_Buttons() throws InterruptedException {
        move(-2 * square, 1);
        turn(-45, 1);
        move(-1.5 * (Math.sqrt(2)) * square, 1);
        turn(-45, 1);
        move(-0.5 * square, 1);
    }
    public void Far_Blue_Buttons() throws InterruptedException {
        move(-square, 1);
        turn(-45, 1);
        move(-2 * (Math.sqrt(2)) * square, 1);
        turn(-45, 1);
        move(-2 * square, 1);
    }
    public void Close_Blue_BlueRamp() throws InterruptedException {
        move(square, 1);
        turn(45, 1);
        move(-Math.sqrt(8) * square, 1);
        turn(90, 1);
    }
    public void Far_Blue_BlueRamp() throws InterruptedException {
        move(2 * square, 1);
        turn(90, 1);
        move(3 * square, 1);
        turn(-45, 1);
        move(.5 * Math.sqrt(2) * square, 1);
    }
    public void Close_Blue_RedRamp() throws InterruptedException {
        move(3 * square, 1);
        turn(-45, 1);
        move(2 * Math.sqrt(2) * square, 1);
    }
    public void Far_Blue_RedRamp() throws InterruptedException {
        move(3.5 * square, 1);
        turn(-45, 1);
    }

//RAMP SEGMENTS

    public void ClearRamp() throws InterruptedException {
        turn(-90, 1);
        move(.75 * square, 1);
        move(-1.5 * square,1);
        move(.75 * square,1);
        turn(90, 1);
    }

    public void ClimbRamp() throws InterruptedException {
        int counter = 0;
        int badData = 0;
        while(ultra.getUltrasonicLevel() < 40) {
            //SOS();
            motorBL.setPower(1);
            motorFL.setPower(1);
            motorBR.setPower(1);
            motorFR.setPower(1);
        }
        while(counter < 10) {
            ssleep(50);
            if (ultra.getUltrasonicLevel() >= 40) {
                badData++;
            }
            counter++;
        }
        if (badData > 4) {
            halt();
        }
        else {
            ClimbRamp();
        }
    }

//BUTTON & CLIMBERS SEGMENT

    public void ButtonClimbers() throws InterruptedException {

    }

//BUTTONS TO RAMP SEGMENTS

    public void RedButtons_RedRamp() throws InterruptedException {
        move(square, 1);
        turn(90, 1);
        move(1.5 * square, 1);
        turn(45,1);
    }
    public void RedButtons_BlueRamp() throws InterruptedException {
        move(4 * square, 1);
        turn(-45,1);
    }
    public void BlueButtons_RedRamp() throws InterruptedException {
        move(square, 1);
        turn(90, 1);
        move(1.5 * square, 1);
        turn(45,1);
    }
    public void BlueButtons_BlueRamp() throws InterruptedException {
        move(4 * square, 1);
        turn(-45,1);
    }

    public void runOpMode() throws InterruptedException {
/*
        motorFL = hardwareMap.dcMotor.get("motor_1");
        motorBL = hardwareMap.dcMotor.get("motor_2");
        motorFR = hardwareMap.dcMotor.get("motor_3");
        motorBR = hardwareMap.dcMotor.get("motor_4");
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
*/

    }
}
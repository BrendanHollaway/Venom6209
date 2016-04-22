package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.DPoint;
import com.qualcomm.ftcrobotcontroller.NewRobotics;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.Range;

import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.BeaconExtension;

import java.util.HashMap;
import java.util.Iterator;
import java.util.TreeMap;
import java.util.concurrent.TimeUnit;

/**
 * Created by Venom6209 on 10/5/2015.
 */
public class AutonomousSegments extends LinearOpModeCV2 {

    UltrasonicSensor ultra;
    UltrasonicSensor frontUltra;
    //protected AdafruitIMU IMU;
    protected AdafruitIMU IMU2;
    Telemetry tele = telemetry;
    LinearOpModeCV parent_op;

    double cm_rotation = 4*Math.PI*2.54;
    double square_per_rot = 60.0/cm_rotation;                  //different units used for measuring distance moved
    double inches = 1.5*Math.PI;
    double degrees = 2000.0/90.0;
    double xPos = 0;
    double yPos = 0;

    public AutonomousSegments()
    {
        super();
    }
    public AutonomousSegments(DcMotor motorBL, DcMotor motorBR, AdafruitIMU IMU)
    {
        this.motorBL = motorBL;
        this.motorBR = motorBR;
        //this.IMU = IMU;
    }

    public AutonomousSegments(DcMotor motorFL, DcMotor motorBL, DcMotor motorFR, DcMotor motorBR, AdafruitIMU IM)

    {
        this.motorFL = motorFL;
        this.motorBL = motorBL;                     //Actually initializes motors
        this.motorFR = motorFR;
        this.motorBR = motorBR;
        this.IMU2 = IM;
    }
    public AutonomousSegments(DcMotor motorFL, DcMotor motorBL, DcMotor motorFR, DcMotor motorBR, AdafruitIMU IMUu, Telemetry telem)
    {
        this.motorFL = motorFL;
        this.motorBL = motorBL;                     //Actually initializes motors
        this.motorFR = motorFR;
        this.motorBR = motorBR;
        this.IMU2 = IMUu;
        tele = telem;
    }
    public AutonomousSegments(DcMotor motorFL, DcMotor motorBL, DcMotor motorFR, DcMotor motorBR, AdafruitIMU IMUu, Telemetry telem, LinearOpMode par_op)
    {
        this.motorFL = motorFL;
        this.motorBL = motorBL;                     //Actually initializes motors
        this.motorFR = motorFR;
        this.motorBR = motorBR;
        this.IMU2 = IMUu;
        tele = telem;
        //this.parent_op = parent_op;
    }
    public AutonomousSegments(DcMotor motorFL, DcMotor motorBL, DcMotor motorFR, DcMotor motorBR, AdafruitIMU IMUu, Telemetry telem, LinearOpModeCV par_op)
    {
        this.motorFL = motorFL;
        this.motorBL = motorBL;                     //Actually initializes motors
        this.motorFR = motorFR;
        this.motorBR = motorBR;
        this.IMU2 = IMUu;
        tele = telem;
        parent_op = par_op;
    }
    public AutonomousSegments(Telemetry telem, LinearOpModeCV par_op)
    {
        tele = telem;
        this.parent_op = par_op;
        this.global_timeout = par_op.global_timeout;
    }

    @Deprecated
    public void Climbers() throws InterruptedException {
        /*if(servoClimberArm == null)
        {
            DbgLog.error("servo climber Arm is null");
            super.stop();
        }*/
        //servoClimberArm.setPosition(0);
        servoClimberHelper.setPosition(0);
        sleep(5000);
        //servoClimberArm.setPosition(1);
        servoClimberHelper.setPosition(1);
        /*sleep(250);
        servoClimberArm.setPosition(1);*/
    }
    public void Close_Blue_Buttons() throws InterruptedException {
        move(1.5, 1);
        tele.addData("we made it: ", "I am tele");
        turn(-14, 0.75);//turn(-23, 1);
        //encoderTurn(45, 1);
        move(5.4 * (Math.sqrt(2)), 0.75);
        turn(-15, 0.75);
        //encoderTurn(-45, 1);
        move(3.2, 0.5);
        halt();
    }
    @Deprecated
    public void move(double squares, double speed) throws InterruptedException        //move in a straight line
    {
        long time = System.currentTimeMillis() + (long) Math.pow(10, 4.1);
        double position = squares / square_per_rot * 1120; //1120 is number of encoder ticks per rotation

        //============RESET THE ENCODERS================
        /*motorFL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        //==========END RESET THE ENCODERS==============
        while(motorFL.getCurrentPosition() != 0 || motorFR.getCurrentPosition() != 0 || motorBR.getCurrentPosition() != 0 || motorBL.getCurrentPosition() != 0) {
            waitOneFullHardwareCycle();
            tele.addData("encoders: ", "resetting");
        }*/
        tele.addData("encoders: ", "done");
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int currentFLPosition = motorFL.getCurrentPosition();                            //measures current encoder value
        int currentFRPosition = motorFR.getCurrentPosition();                            //measures current encoder value
        int currentBLPosition = motorBL.getCurrentPosition();                            //measures current encoder value
        int currentBRPosition = motorBR.getCurrentPosition();                            //measures current encoder value*/
        int currentEncoder = 0; //=mid2(currentBLPosition, currentBRPosition, currentFLPosition, currentFRPosition);
        DbgLog.error("position = " + position);
        while(Math.abs(currentEncoder) < Math.abs(position) && System.currentTimeMillis() < time) {  //moves until encoders change by value inputted
            motorFL.setPower(Math.signum(position) * Math.abs(speed));
            motorBL.setPower(Math.signum(position) * Math.abs(speed));                 //takes sign of position, so sign of speed does not matter
            motorFR.setPower(Math.signum(position) * Math.abs(speed));
            motorBR.setPower(Math.signum(position) * Math.abs(speed));
            //motorFL and motorBR's encoder values are negative. Don't ask. Idk why.
            currentEncoder = mid2(motorBL.getCurrentPosition() - currentBLPosition, motorFL.getCurrentPosition() - currentFLPosition, motorFR.getCurrentPosition() - currentFRPosition, motorBR.getCurrentPosition() - currentBRPosition);
            DbgLog.error("" + currentEncoder);
            DbgLog.error("FR: "+ String.format("%d,FL: %d", motorFR.getCurrentPosition() - currentFRPosition, motorFL.getCurrentPosition() - currentFLPosition));
            DbgLog.error("BR: " + String.format("%d,BL: %d", motorBR.getCurrentPosition() - currentBRPosition, motorBL.getCurrentPosition() - currentBLPosition));
            //tele.addData("PowFR: ", String.format("%.2f,FL: %.2f", motorFR.getPower(), motorFL.getPower()));
            //tele.addData("PowBR: ", String.format("%.2f,BL: %.2f", motorBR.getPower(), motorBL.getPower()));
            //DbgLog.error("Encoder BR, BL, FR, FL" + motorBR.getCurrentPosition() + " " + motorBL.getCurrentPosition() + " " + motorFR.getCurrentPosition() + " " + motorFL.getCurrentPosition());
            this.waitOneFullHardwareCycle();
            //waitOneFullHardwareCycle();
        }
        tele.addData("made it to the end of loop", " done");
        halt();
    }
    public int mid2(int enc1, int enc2, int enc3, int enc4)
    {
        if(!this.opModeIsActive())
            return -1;
        enc1 = Math.abs(enc1);
        enc2 = Math.abs(enc2);
        enc3 = Math.abs(enc3);
        enc4 = Math.abs(enc4);
        int min = Math.min(enc1, Math.min(enc2, Math.min(enc3, enc4)));
        int max = Math.max(enc1, Math.max(enc2, Math.max(enc3, enc4)));
        return (enc1 + enc2 + enc3 + enc4 - max - min) / 2;
    }
    public void turn(double deg, double speed) throws InterruptedException{
        turn(deg, speed, 1, Double.MAX_VALUE);
    }
    public void turn(double deg, double speed, double timeout) throws InterruptedException{
        turn(deg, speed, 1, timeout);
    }
    public void turn(double deg, double speed, double tolerance, double timeout) throws InterruptedException
    {
        if(!this.opModeIsActive())
            return;
        speed = Range.clip(Math.abs(speed), -1, 1);
        tele.addData("turn has ", "begun");
        tele.addData("deg: ", getGyroYaw());
        timeout = timeout * Math.pow(10, 3) + System.currentTimeMillis();
        //halt();
        DcMotor.Direction FR = motorFR.getDirection();
        DcMotor.Direction BR = motorBR.getDirection();
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        if(!this.opModeIsActive())
            return;
        //Range.clip(Math.abs(speed), -1, 1);
        double gyro = getGyroYaw();
        tele.addData("gryo:", String.format("%.2f, deg: %.2f", gyro, deg));
        deg += gyro;
        deg %= 180;
        tele.addData("deg:", deg);
        //deg now between -180 and 180
        String format;
        DbgLog.error(String.format("start: deg:%.2f, gyro:%.2f", deg, getGyroYaw()));
        if(gyro < deg)
        {
            if(!this.opModeIsActive())
                return;
            while(gyro < deg - 3 && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout && this.opModeIsActive())
            {
                format = String.format("inside fast loop, gyro:%.2f, deg:%.2f", gyro, deg);
                tele.addData("tele: ", format);
                DbgLog.error(format);
                motorFR.setPower(-speed);
                motorFL.setPower(speed);
                motorBR.setPower(-speed);
                motorBL.setPower(speed);
                gyro = getGyroYaw();
            }
            if(!this.opModeIsActive())
                return;
            /*while(gyro < deg && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout && !this.opModeIsActive())
            {
                format = String.format("inside slow loop, gyro:%.2f, deg:%.2f", gyro, deg);
                tele.addData("tele: ", format);
                DbgLog.error(format);
                motorFR.setPower(-0.4);
                motorFL.setPower(0.4);
                motorBR.setPower(-0.4);
                motorBL.setPower(0.4);
                gyro = getGyroYaw();
            }*/
        }
        else
        {
            if(!this.opModeIsActive())
                return;
            while(gyro > deg + 3 && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout && this.opModeIsActive())
            {
                format = String.format("inside fast loop, gyro:%.2f, deg:%.2f", gyro, deg);
                tele.addData("tele: ", format);
                DbgLog.error(format);
                motorFR.setPower(speed);
                motorFL.setPower(-speed);
                motorBR.setPower(speed);
                motorBL.setPower(-speed);
                gyro = getGyroYaw();
            }
            if(!this.opModeIsActive())
                return;
            /*while(gyro > deg && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout && !this.opModeIsActive())
            {
                format = String.format("inside slow loop, gyro:%.2f, deg:%.2f", gyro, deg);
                tele.addData("tele: ", format);
                DbgLog.error(format);
                motorFR.setPower(0.4);
                motorFL.setPower(-0.4);
                motorBR.setPower(0.4);
                motorBL.setPower(-0.4);
                gyro = getGyroYaw();
            }*/
        }
        gyro = getGyroYaw();
        if(!this.opModeIsActive())
            return;
        DbgLog.error(String.format("end: deg:%.2f, gyro:%.2f",deg, gyro));
        this.target_heading = gyro;
        //if(Math.abs(getGyroYaw() - deg) > tolerance && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout)
          //  turn(deg - getGyroYaw(), 0.8, tolerance * 1.2, (timeout - System.currentTimeMillis()) / Math.pow(10, 3)));
        motorFR.setDirection(FR);
        motorBR.setDirection(BR);
        if(!this.opModeIsActive())
            return;
        halt();
        resetPID();
        if(!this.opModeIsActive())
            return;
        DbgLog.error(String.format("post-halt: deg:%.2f, gyro:%.2f", deg, getGyroYaw()));
        if(Math.abs(deg-  getGyroYaw()) > 5 && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout)
            turn(deg-getGyroYaw(), 0.05);
    }
    public void timed_turn(double deg, double speed, double tolerance, double timeout) throws InterruptedException
    {
        if(!this.opModeIsActive())
            return;
        speed = Range.clip(Math.abs(speed), -1, 1);
        tele.addData("turn has ", "begun");
        tele.addData("deg: ", getGyroYaw());
        timeout = timeout * Math.pow(10, 3) + System.currentTimeMillis();
        //halt();
        DcMotor.Direction FR = motorFR.getDirection();
        DcMotor.Direction BR = motorBR.getDirection();
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        if(!this.opModeIsActive())
            return;
        //Range.clip(Math.abs(speed), -1, 1);
        double gyro = getGyroYaw();
        tele.addData("gryo:", String.format("%.2f, deg: %.2f", gyro, deg));
        deg += gyro;
        deg %= 180;
        tele.addData("deg:", deg);
        //deg now between -180 and 180
        String format;
        DbgLog.error(String.format("start: deg:%.2f, gyro:%.2f", deg, getGyroYaw()));
        resetStartTime();
        if(gyro < deg)
        {
            if(!this.opModeIsActive())
                return;
            while(gyro < deg - 20 && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout && this.opModeIsActive())
            {
                format = String.format("inside fast loop, gyro:%.2f, deg:%.2f", gyro, deg);
                tele.addData("tele: ", format);
                DbgLog.error(format);
                motorFR.setPower(-speed / 3.0);
                motorFL.setPower(speed / 3.0);
                motorBR.setPower(-speed);
                motorBL.setPower(speed);
                gyro = getGyroYaw();
            }
            DbgLog.error("Done with fast");
            while(gyro < deg - 3 && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout && this.opModeIsActive())
            {
                format = String.format("inside fast loop, gyro:%.2f, deg:%.2f", gyro, deg);
                tele.addData("tele: ", format);
                DbgLog.error(format);
                motorFR.setPower(-.06);
                motorFL.setPower(.06);
                motorBR.setPower(-.12);
                motorBL.setPower(.12);
                gyro = getGyroYaw();
            }
            DbgLog.error("Done with slow");
            if(!this.opModeIsActive())
                return;
            /*while(gyro < deg && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout && !this.opModeIsActive())
            {
                format = String.format("inside slow loop, gyro:%.2f, deg:%.2f", gyro, deg);
                tele.addData("tele: ", format);
                DbgLog.error(format);
                motorFR.setPower(-0.4);
                motorFL.setPower(0.4);
                motorBR.setPower(-0.4);
                motorBL.setPower(0.4);
                gyro = getGyroYaw();
            }*/
        }
        else
        {
            if(!this.opModeIsActive())
                return;
            while(gyro > deg + 20 && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout && this.opModeIsActive())
            {
                format = String.format("inside fast loop, gyro:%.2f, deg:%.2f", gyro, deg);
                tele.addData("tele: ", format);
                DbgLog.error(format);
                motorFR.setPower(speed / 3.0);
                motorFL.setPower(-speed / 3.0);
                motorBR.setPower(speed);
                motorBL.setPower(-speed);
                gyro = getGyroYaw();
            }
            DbgLog.error("Done with fast");
            while(gyro > deg + 3 && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout && this.opModeIsActive())
            {
                format = String.format("inside slow loop, gyro:%.2f, deg:%.2f", gyro, deg);
                tele.addData("tele: ", format);
                DbgLog.error(format);
                motorFR.setPower(-.06);
                motorFL.setPower(.06);
                motorBR.setPower(-.12);
                motorBL.setPower(.12);
                gyro = getGyroYaw();
            }
            DbgLog.error("Done with slow");
            if(!this.opModeIsActive())
                return;
            /*while(gyro > deg && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout && !this.opModeIsActive())
            {
                format = String.format("inside slow loop, gyro:%.2f, deg:%.2f", gyro, deg);
                tele.addData("tele: ", format);
                DbgLog.error(format);
                motorFR.setPower(0.4);
                motorFL.setPower(-0.4);
                motorBR.setPower(0.4);
                motorBL.setPower(-0.4);
                gyro = getGyroYaw();
            }*/
        }
        gyro = getGyroYaw();
        if(!this.opModeIsActive())
            return;
        DbgLog.error(String.format("end: deg:%.2f, gyro:%.2f",deg, gyro));
        this.target_heading = gyro;
        //if(Math.abs(getGyroYaw() - deg) > tolerance && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout)
        //  turn(deg - getGyroYaw(), 0.8, tolerance * 1.2, (timeout - System.currentTimeMillis()) / Math.pow(10, 3)));
        motorFR.setDirection(FR);
        motorBR.setDirection(BR);
        if(!this.opModeIsActive())
            return;
        halt();
        resetPID();
        if(!this.opModeIsActive())
            return;
        DbgLog.error(String.format("post-halt: deg:%.2f, gyro:%.2f", deg, getGyroYaw()));
        if(Math.abs(deg-  getGyroYaw()) > 5 && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout)
            turn(deg-getGyroYaw(), 0.05);
    }
    public void ssleep(long ms) throws InterruptedException                 //method for sleeping
    {
        try {
            sleep(ms);
        }
        catch (Exception E){}
    }
    public void move2(double squares) throws InterruptedException
    {
        double position = squares / square_per_rot * 1120;
        double target = position + motorFL.getCurrentPosition();
        /*DcMotor.Direction fl = motorFL.getDirection();
        DcMotor.Direction fr = motorFR.getDirection();
        DcMotor.Direction bl = motorBL.getDirection();
        DcMotor.Direction br = motorBR.getDirection();*/
        //============RESET THE ENCODERS================
        motorFL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        //==========END RESET THE ENCODERS==============
        while(motorFL.getCurrentPosition() != 0 || motorFR.getCurrentPosition() != 0 || motorBR.getCurrentPosition() != 0 || motorBL.getCurrentPosition() != 0) {
            this.waitOneFullHardwareCycle();
        }
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        this.waitOneFullHardwareCycle();
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFL.setTargetPosition((int) position);
        motorFR.setTargetPosition((int) position);
        motorBL.setTargetPosition((int) position);
        motorBR.setTargetPosition((int)position);
        motorFL.setPower(0.8);
        motorFR.setPower(0.8);
        motorBL.setPower(0.8);
        motorBR.setPower(0.8);
        while(Math.abs(motorFL.getCurrentPosition() - target) < 10 && Math.abs(motorFR.getCurrentPosition() - target) < 10 && Math.abs(motorBR.getCurrentPosition() - target) < 10 && Math.abs(motorBL.getCurrentPosition() - target) < 10 )//Math.abs(motorFL.getCurrentPosition()) < Math.abs(target) && Math.abs(motorFR.getCurrentPosition()) < target && Math.abs(motorFL.getCurrentPosition()) < target && Math.abs(motorFL.getCurrentPosition()) < target)
        {
            this.waitOneFullHardwareCycle();
            motorFL.setPower(0.8);
            motorFR.setPower(0.8);
            motorBL.setPower(0.8);
            motorBR.setPower(0.8);
            tele.addData("FR: ", String.format("%d,FL: %d", motorFR.getCurrentPosition(), motorFL.getCurrentPosition())); // 5000, -7500
            tele.addData("BR: ", String.format("%d,BL: %d", motorBR.getCurrentPosition(), motorBL.getCurrentPosition())); // -5000, 7500
            tele.addData("PowFR: ", String.format("%.2f,FL: %.2f", motorFR.getPower(), motorFL.getPower()));
            tele.addData("PowBR: ", String.format("%.2f,BL: %.2f", motorBR.getPower(), motorBL.getPower()));
        }
        halt();
    }
    /*public void left(double position, double speed) throws InterruptedException
    {
        speed = Range.clip(speed, -1, 1);
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
        speed = Range.clip(speed, -1, 1);
        while(Math.abs(motorFR.getCurrentPosition()) < position ) {
            motorFL.setPower(0);
            motorBL.setPower(0);
            motorFR.setPower(Math.signum(position) * Math.abs(speed));
            motorBR.setPower(Math.signum(position) * Math.abs(speed));
        }
        halt();
    }*/
    public void halt() throws InterruptedException{
        if(!this.opModeIsActive())
            return;
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        this.waitOneFullHardwareCycle();
    }
    public double getGyroYaw() throws InterruptedException{
        if(!this.opModeIsActive())
            return -1;
        if(IMU == null) {
            DbgLog.error(" IMU is null");
            return -1;
        }
        IMU.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return yawAngle[0];
    }
    public double getGyroPitch() throws InterruptedException{
        if(!this.opModeIsActive())
            return -1;
        if(IMU == null) {
            DbgLog.error(" IMU is null");
            return -1;
        }
        IMU.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return pitchAngle[0];
    }
    public double getGyroRoll() throws InterruptedException{
        if(!this.opModeIsActive())
            return -1;
        if(IMU == null) {
            DbgLog.error(" IMU is null");
            return -1;
        }
        IMU.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return rollAngle[0];
    }
    public int squares_to_Encoder(double squares)
    {
        return (int)(squares / square_per_rot * 1120);
    }


//RED INITIAL SEGMENTS


    public void Close_Red_Buttons() throws InterruptedException {
        move2(0.75);// , 1);
        telemetry.addData("we made it: ", "yay");
        halt();
        ssleep(1000);
        turn(14, 0.75);//turn(-23, 1);
        //encoderTurn(45, 1);
        move2(4.25 * (Math.sqrt(2)));// , 0.75);
        turn(18, 0.75);
        //encoderTurn(-45, 1);
        move2(0.25);// , 0.5);
        halt();
    }
    public void Far_Red_Buttons() throws InterruptedException {
        move(- 1, 1);
        turn(45, 1);
        //encoderTurn(45, 1);
        move(-2 * (Math.sqrt(2)) , 1);
        turn(45, 1);
        //encoderTurn(45, 1);
        move(-2 , 1);
    }
    public void Close_Red_RedRamp() throws InterruptedException {
        move( 1, 1);
        turn(-45, 1);
        //encoderTurn(-45, 1);
        move(-Math.sqrt(2) , 1);
        turn(-90, 1);
        //encoderTurn(-90, 1);
    }
    public void Far_Red_RedRamp() throws InterruptedException {
        move(2 , 1);
        turn(-90, 1);
        //encoderTurn(-90, 1);
        move(3 , 1);
        turn(45, 1);
        //encoderTurn(45, 1);
        move(.5 * Math.sqrt(2) , 1);
    }
    public void Close_Red_BlueRamp() throws InterruptedException {
        move(3 , 1);
        turn(45, 1);
        //encoderTurn(45, 1);
        move(2 * Math.sqrt(2) , 1);
    }
    public void Far_Red_BlueRamp() throws InterruptedException {
        move(3.5, 1);
        turn(45, 1);
        //encoderTurn(45, 1);
    }
    //===========================================================
    //========================PID CONTROL========================
    //===========================================================
    protected double kP=0.06;
    protected double kI=0.02;
    protected double kD=0.005;
    double dt;
    double prevError;
    double error = 0;
    double iError = 0;
    double dError;
    double time = getRuntime();
    double target_heading;
    double prevGyro = 1000;
    double prevPID = 1000;
    TreeMap<Double, Double> time_displacement = new TreeMap<Double, Double>();
    public double tune_PID() throws InterruptedException
    {
        return tune_PID(10000);
    }
    public double tune_PID(double timeout) throws InterruptedException
    {
        long timer = (long) (timeout * Math.pow(10, 3)) + System.currentTimeMillis();
        double time_per_run = 10;
        double kP = 0.00005; // start value. It IS too small.
        //double max;
        target_heading = 10;
        resetPID();
        while(!isTuned(kP, time_per_run) && System.currentTimeMillis() < timer)
        {
            kP *= 2;
            target_heading = (target_heading == 90? 0 : 90);
        }
        kP /= 2;
        while(!isTuned(kP, time_per_run) && System.currentTimeMillis() < timer)
        {
            kP *= 1.1;
            target_heading = (target_heading == 90? 0 : 90);
        }
        kP /= 1.1;
        while(!isTuned(kP, time_per_run) && System.currentTimeMillis() < timer)
        {
            kP *= 1.01;
            target_heading = (target_heading == 90? 0 : 90);
        }
        kP /= 1.01;
        Iterator<Double> it = time_displacement.keySet().iterator();
        double Tu = 0; // Tu is the period of the oscillation. T meaning period in physics.
        double previous = it.next();
        //finding the average period
        //note: next-previous is one half period... so needs to be doubled at the end
        while (it.hasNext()) {
            double next = it.next();
            Tu += next - previous;
            previous = next;
        }
        Tu = Tu * 2 / (time_displacement.size() - 1); // calculate period based off of some number of half-periods
        this.kP = 0.6 * kP;
        this.kI = 2 * this.kP / Tu;
        this.kD = this.kP * Tu / 8;
        return time_displacement.get(previous); // returns the amplitude of the oscillations
    }
    public boolean isTuned(double kP, double timeout) throws InterruptedException
    {
        //TODO: add debug log info for debugging
        time_displacement = new TreeMap<Double, Double>();
        long timer = (long) (timeout * Math.pow(10, 3)) + System.currentTimeMillis();
        resetStartTime();
        double PID_change;
        double right;
        double left;
        double new_yaw = getGyroYaw();
        resetPID();
        double sign = Math.signum(get_PID(kP, 0, 0));
        while(System.currentTimeMillis() < timer)
        {
            while(new_yaw == getGyroYaw())
            {
                this.waitOneFullHardwareCycle();
            }
            new_yaw = getGyroYaw();
            PID_change = get_PID(new_yaw, kP, 0, 0);
            if(sign != Math.signum(PID_change))
            {
                time_displacement.put(getRuntime(), error);
                sign *= -1;
            }
            //keep the absolute value of the motors above 0.3 and less than 0.7
            right = Range.clip(PID_change, -1, 1);
            left = -right;
            setRightPower(right);
            setLeftPower(left);
            DbgLog.error(String.format("k*error: %.2f, error: %.2f", kP * error, error));
            DbgLog.error(String.format("gyro:%.2f, target:%.2f, PID (right):%.2f", getGyroYaw(), target_heading, PID_change));
        }
        DbgLog.error(String.format("time_displacement.size(): %d", time_displacement.size()));
        DbgLog.error(time_displacement.toString());
        if(time_displacement.size() < 3)
            return false;
        double max = Double.MIN_VALUE;
        double min = Double.MAX_VALUE;
        Iterator<Double> it = time_displacement.keySet().iterator();
        while(it.hasNext())
        {
            double next = Math.abs(time_displacement.get(it.next())); // measuring amplitude differentials.
            max = Math.max(max, next);
            min = Math.min(min, next);
        }
        if(max - min > 1) // amplitude still decreasing too much over time. needs to run again.
            return false;
        return true;
    }
    public void PID_turn(double deg) throws InterruptedException
    {
        PID_turn(deg, 3.0, 10);
    }
    public void PID_turn(double deg, double tolerance) throws InterruptedException
    {
        PID_turn(deg, tolerance, 10);
    }
    public void PID_turn(double deg, double tolerance, double timer) throws InterruptedException
    {
        PID_turn(deg, tolerance, timer, 1);
    }
    public void PID_turn(double deg, double tolerance, double timer, double speed_divisor) throws InterruptedException
    {
        DbgLog.error("PID_turn begin, deg: " + String.format("%.2f, gyro: %.2f",deg, getGyroYaw()));
        double safety_time = getRuntime() + timer;
        double kP = 0.009;
        double kI = 0.0001;
        double kD = 0.0009;
        double PID_change;
        double right;
        double left;
        //double max;
        double target = deg + getGyroYaw();
        target_heading = target;
        while(/*Math.abs(getGyroYaw() - target_heading) > tolerance*/ this.opModeIsActive() && getRuntime() < safety_time && System.currentTimeMillis() < global_timeout)
        {
            PID_change = get_PID(kP, kI, kD);
            //keep the absolute value of the motors above 0.3 and less than 0.7
            right = (Math.signum(PID_change) * Range.clip(Math.abs(PID_change), -1, 1) / speed_divisor);
            left = -right;
            setRightPower(right);
            setLeftPower(left);
            DbgLog.error(String.format("k*error: %.5f, k*dError: %.5f, k*iError: %.5f", kP * error, kD * dError, kI * iError));
            DbgLog.error(String.format("error: %.2f, dError: %.2f, iError: %.2f", error, dError, iError));
            DbgLog.error(String.format("gyro:%.2f, target:%.2f, right:%.5f", getGyroYaw(), target_heading, right));
            tele.addData("gyro: ", getGyroYaw());
        }
        tele.addData("done", " ");
        DbgLog.error(String.format("gyro:%.2f, target:%.2f, right:%.5f", getGyroYaw(), target_heading, 0.0));
        if(!this.opModeIsActive())
            return;
        halt();
        halt();
        halt();
        double gyro = getGyroYaw();
        DbgLog.error(String.format("gyro: %.2f, target: %.2f, tolerance: %.2f", gyro, target, tolerance));
        if((Math.abs(gyro - target) > tolerance) && System.currentTimeMillis() < safety_time)
            PID_turn(target - gyro, tolerance, safety_time - getRuntime(), speed_divisor * 1.11);
    }
    public double PID_turn_time(double deg, double timer) throws InterruptedException
    {
        return PID_turn_time(deg, timer, false);
    }
    public double PID_turn_time(double deg, double timer, boolean only_fine) throws InterruptedException
    {
        DbgLog.error("PID_turn begin, deg: " + String.format("%.2f, gyro: %.2f",deg, getGyroYaw()));
        double safety_time = getRuntime() + timer;
        double kP = 0.005; //.04
        double kI = 0.0015;
        double kD = 0.0015;
        double PID_change;
        double right;
        double left;
        //double max;
        double target = deg + getGyroYaw();
        target_heading = target;
        resetPID();
        while(!only_fine && this.opModeIsActive() && getRuntime() < safety_time / 1.5 && System.currentTimeMillis() < global_timeout)
        {
            tele.addData("driving: ", "turning");
            PID_change = get_PID(kP, kI, kD);
            right = Range.clip(PID_change, -1, 1);
            left = -right;
            setRightPower(right);
            setLeftPower(left);
            DbgLog.error(String.format("k*error: %.5f, k*dError: %.5f, k*iError: %.5f", kP * error, kD * dError, kI * iError));
            DbgLog.error(String.format("error: %.2f, dError: %.2f, iError: %.2f", error, dError, iError));
            DbgLog.error(String.format("gyro:%.2f, target:%.2f, right:%.5f", getGyroYaw(), target_heading, right));
            tele.addData("gyro: ", getGyroYaw());
            tele.addData("PID: ", String.format("k*error: %.5f, k*dError: %.5f, k*iError: %.5f", kP * error, kD * dError, kI * iError));
            tele.addData("speed: ", right);
        }
        resetPID();
        DbgLog.error(String.format("current: %.2f, target: %.2f", getGyroYaw(), target_heading));
        kP = 0.08;
        kI = 0.04;
        kD = 0.01;
        while(this.opModeIsActive() && getRuntime() < safety_time && System.currentTimeMillis() < global_timeout)
        {
            tele.addData("driving: ", "turning");
            PID_change = get_PID(kP, kI, kD) / 8.0;
            //keep the absolute value of the motors above 0.3 and less than 0.7
            right = Range.clip(PID_change, -1, 1);
            left = -right;
            setRightPower(right);
            setLeftPower(left);
            DbgLog.error(String.format("k*error: %.5f, k*dError: %.5f, k*iError: %.5f", kP * error, kD * dError, kI * iError));
            DbgLog.error(String.format("error: %.2f, dError: %.2f, iError: %.2f", error, dError, iError));
            DbgLog.error(String.format("gyro:%.2f, target:%.2f, right:%.5f", getGyroYaw(), target_heading, right));
            tele.addData("gyro: ", getGyroYaw());
            tele.addData("PID: ", String.format("k*error: %.5f, k*dError: %.5f, k*iError: %.5f", kP * error, kD * dError, kI * iError));
            tele.addData("speed: ", right);
        }
        resetPID();
        halt();
        halt();
        halt();
        tele.addData("done", " with PID_turn_time");
        return target - getGyroYaw();
    }
    //Polar-written movement. Useful for converting old methods to new format
    public void PID_move_displacement_polar(double r, double theta, double speed) throws InterruptedException {
        PID_move_displacement_polar(r, theta, speed, 12);
    }
    double enc_tolerance = 350;
    public void PID_move_displacement_polar(double r_encoder_ticks, double theta, double speed, double timeout) throws InterruptedException
    {
        double target_x = xPos + r_encoder_ticks * Math.cos(Math.toRadians(theta));//((r > 0)? Math.abs(r * Math.cos(theta)): -Math.abs(r * Math.cos(theta)));
        double target_y = yPos + r_encoder_ticks * Math.sin(Math.toRadians(theta));// ((r > 0)? Math.abs(r * Math.sin(theta)): -Math.abs(r * Math.sin(theta)));
        PID_move_displacement_cartesian(target_x, target_y, speed, timeout);
    }
    public void PID_move_displacement_cartesian(double target_x, double target_y, double speed, double timeout) throws InterruptedException {
        speed = Range.clip(speed, -1, 1);
        double max;
        resetStartTime();
        timeout = System.currentTimeMillis() + timeout * Math.pow(10, 3);
        double PID_change;
        double right;
        double left;
        //converting polar to rectangular
        double gyro_radians;
        double gyro_yaw;
        double new_heading;
        double vector_x;
        double vector_y;
        double oldEncoder;
        double currEncoder = 0;
        double dE; //delta encoder, the change in average encoder value between runs
        double dEx = 0;
        double dEy = 0;
        boolean reversed = false;
        vector_x = target_x - xPos;
        vector_y = target_y - yPos;
        double theta = Math.atan2(vector_y, vector_x);
        double kP = 0.08;
        double kI = this.kI;
        double kD = 0.001;
        /*
        motorFL.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
         */
        motorFL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int currentFLPosition = motorFL.getCurrentPosition();                            //measures current encoder value
        int currentFRPosition = motorFR.getCurrentPosition();                            //measures current encoder value
        int currentBLPosition = motorBL.getCurrentPosition();                            //measures current encoder value
        int currentBRPosition = motorBR.getCurrentPosition();                            //measures current encoder value
        while((Math.abs(target_x-xPos) > enc_tolerance || Math.abs(target_y-yPos) > enc_tolerance) && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout)
        {
            gyro_yaw = getGyroYaw();
            DbgLog.error(String.format("gyro: %.2f", gyro_yaw));
            reversed = false;
            //TARGET HEADING CALCULATIONS
            vector_x = target_x - xPos;
            vector_y = target_y - yPos;
            new_heading = Math.atan2(vector_y, vector_x);
            target_heading = Math.toDegrees(new_heading); // rad -> deg; range: -180 to 180
            if(Math.abs(gyro_yaw - target_heading) > 90) // this happens if the robot overshoots, and thus reversing is the fastest way to the target.
            {
                target_heading = -target_heading;
                reversed = true;
            }
            //ECNCODER VALUE CALCULATIONS
            oldEncoder = currEncoder;
            //MotorFR is in here twice bc the encoder on MotorBR doesn't work currently
            currEncoder = mid2(motorBL.getCurrentPosition() - currentBLPosition, motorFL.getCurrentPosition() - currentFLPosition, motorBR.getCurrentPosition() - currentBRPosition, motorBR.getCurrentPosition() - currentBRPosition /*motorBR.getCurrentPosition() - 0 currentBRPosition*/);
            dE = currEncoder - oldEncoder;
            gyro_radians = Math.toRadians(gyro_yaw);
            dEx = dE * Math.cos(gyro_radians);
            dEy = dE * Math.sin(gyro_radians);
            // ACCELERATION VALUE CALCULATIONS
            //IMU.getAccel(accs);
            //old_y_acc = curr_y_acc;
            //curr_y_acc = accs[1];
            //VELOCITY
            //y_vel += 0.5 * (old_y_acc + curr_y_acc) * dt;
            //CALCULATE POSITION
            xPos += dEx;
            yPos += dEy;
            //CALCULATE SPEED
            PID_change = get_PID(kP, this.kI, kD);
            right = speed + PID_change;
            left = speed / 1.4 - PID_change;
            max = Math.max(Math.abs(right), Math.abs(left));
            //This standardizes the speeds, so that they are correct relative to each other,
            //and that one of the two will be equivalent, and neither greater, than @param speed
            right /= max;
            right *= speed;
            left /= max;
            left *= speed;
            if(reversed) // this happens if the robot overshoots, and thus reversing is the fastest way to the target.
            {
                right *= -1;
                left *= -1;
            }
            setRightPower(right);
            setLeftPower(left);
            this.waitOneFullHardwareCycle();
            DbgLog.error(String.format("x:%.2f, y:%.2f, target_x: %.2f, target_y: %.2f", xPos, yPos, target_x, target_y));
            DbgLog.error(String.format("gyro: %.2f, target_heading: %.2f", gyro_yaw, target_heading));
            //DbgLog.error(String.format("error: %.2f, iError: %.2f, dError: %.2f", error, iError, dError));
        }
        turn(theta - getGyroYaw(), 0.5);
        vector_x = target_x - xPos;
        vector_y = target_y - yPos;
        PID_move(Math.hypot(vector_x, vector_y), target_heading, 0.4, false, 4000);
        DbgLog.error("done with cartesian move");
    }
    public void PID_move(double encoder, double target_heading, double speed, boolean enableCamera) throws InterruptedException
    {
        PID_move(encoder, target_heading, speed, enableCamera, 12000L);
    }
    public void PID_move(double encoder, double target_heading, double speed, boolean enableCamera, long timeout) throws InterruptedException
    {
        DbgLog.error("PID_move begin, target_heading: " + String.format("%.2f, gyro: %.2f",target_heading, getGyroYaw()));
        speed = Range.clip(Math.abs(speed), 0, 1);
        this.target_heading = target_heading;
        //encoder *= 2;   // after I fixed some encoders, I realized that the
                        // robot was moving half the intended distance due to an error.
                        // this saves me the time of doubling every value from now on. Whoops :/
        //motorFL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //motorFL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //All errors are in terms of heading: the robot's yaw
        double max;
        double gyro_yaw;
        resetStartTime();
        double safety_time = System.currentTimeMillis() + timeout;
        double start_time = System.currentTimeMillis();
        double PID_change;
        double right;
        double left;
        double kP = this.kP;
        double kI = this.kI;
        double kD = this.kD;
        motorFL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int currentFLPosition = motorFL.getCurrentPosition();                            //measures current encoder value
        int currentFRPosition = motorFR.getCurrentPosition();                            //measures current encoder value
        int currentBLPosition = motorBL.getCurrentPosition();                            //measures current encoder value
        int currentBRPosition = motorBR.getCurrentPosition();                            //measures current encoder value*/
        int currentEncoder = 0; //=mid2(currentBLPosition, currentBRPosition, currentFLPosition, currentFRPosition);
        resetPID();
        while(Math.abs(currentEncoder) < Math.abs(encoder) && System.currentTimeMillis() < safety_time && System.currentTimeMillis() < global_timeout)  //moves until encoders change by value inputted
        {
            // when the camera is enabled, target_heading changes rapidly, leading to massive kD values.
            // thus, a PI controller is more apt to maintain a steady course.
            // kP is increased to account for the decrease in "PID_change"
            // from the dError term. This also reduces the influence of the I term,
            // which is necessarily reduced for the same reason- constant fluctuations
            // lead to inaccurate resutls.
            if(!enableCamera)
                PID_change = get_PID();
            else {
                kP = this.kP * 1.5;
                kI = this.kI / 1.4;
                kD = this.kD;
                PID_change = get_PID(kP, kI, kD) / 3.0;
            }
            right = speed + PID_change;
            left = speed / 3.0 - PID_change;
            max = Math.max(Math.abs(right), Math.abs(left));
            //This standardizes the speeds, so that they are correct relative to each other,
            //and that one of the two will be equivalent, and neither greater, than "speed"
            right /= max;
            right *= speed;
            left /= max;
            left *= speed;
            if(encoder < 0) {
                setRightPower(-right);
                setLeftPower(-left);
            }
            else {
                setRightPower(right);
                setLeftPower(left);
            }
            gyro_yaw = getGyroYaw();
            //DEBUG LOGGING
            tele.addData("a: ", String.format("k*error: %.2f, k*dError: %.2f, k*iError: %.2f", kP * error, kD * dError, kI * iError));
            tele.addData("b: ", String.format("left: %.2f, right: %.2f", left, right));
            tele.addData(" ", String.format("gyro yaw: %.2f, target: %.2f", gyro_yaw, this.target_heading));
            DbgLog.error(String.format("k*error: %.2f, k*dError: %.2f, k*iError: %.2f", kP * error, kD * dError, kI * iError));
            DbgLog.error(String.format("left: %.2f, right: %.2f, gyro: %.2f, target: %.2f", left, right, gyro_yaw, this.target_heading));
            currentEncoder = mid2(motorBL.getCurrentPosition() - currentBLPosition, motorFL.getCurrentPosition() - currentFLPosition, motorBR.getCurrentPosition() - currentBRPosition, motorBR.getCurrentPosition() - currentBRPosition /*motorBR.getCurrentPosition() - 0 currentBRPosition*/);
            DbgLog.error("currentEncoder: " + currentEncoder + ", target_enc: " + String.format("%.2f",encoder));
            DbgLog.error("FR: "+ String.format("%d, FL: %d", motorFR.getCurrentPosition() - currentFRPosition, motorFL.getCurrentPosition() - currentFLPosition));
            DbgLog.error("BR: " + String.format("%d, BL: %d", motorBR.getCurrentPosition() - currentBRPosition, motorBL.getCurrentPosition() - currentBLPosition));
            //END DEBUG LOGGING

            this.waitOneFullHardwareCycle();
            /*if(enableCamera && beacon.getAnalysisMethod().equals(Beacon.AnalysisMethod.FAST))
            {
                DbgLog.error("Beacon Color" + beacon.getAnalysis().getColorString());
                DbgLog.error("Beacon Confidence" + beacon.getAnalysis().getConfidenceString());
                DbgLog.error("Beacon Location (Center)" + String.format("x: %.2f y: %.2f", beacon.getAnalysis().getCenter().y, beacon.getAnalysis().getCenter().x));
                DbgLog.error("Relative " + String.format("x: %.2f y: %.2f", beacon.getAnalysis().getCenter().y / 480.0, beacon.getAnalysis().getCenter().x / 864.0));
                DbgLog.error("New Heading: " + NewRobotics.calculate_heading(new DPoint(beacon.getAnalysis().getCenter().y, beacon.getAnalysis().getCenter().x)));
                if(beacon.getAnalysis().getConfidence() > 0.2) { // beacon.getAnalysis().getConfidence() > 0.1 && <--- use this if COMPLEX analysis is on
                    double getHeading = -getHeading(System.currentTimeMillis() - start_time);
                    this.target_heading = gyro_yaw + getHeading;
                    DbgLog.error(String.format("new heading target1: %.2f, new2:%.2f, old heading: %.2f", this.target_heading,
                            gyro_yaw - NewRobotics.calculate_heading(new DPoint(beacon.getAnalysis().getCenter().y, beacon.getAnalysis().getCenter().x), (System.currentTimeMillis() - start_time) / Math.pow(10, 3)),
                            gyro_yaw));
                }
            }
            else if(enableCamera && beacon.getAnalysis().isBeaconFound()) { // beacon.getAnalysis().getConfidence() > 0.1 && <--- use this if COMPLEX analysis is on
                this.target_heading = gyro_yaw + getHeading(System.currentTimeMillis() - start_time);
                DbgLog.error(String.format("new heading target: %.2f, old heading: %.2f", gyro_yaw +  getHeading(System.currentTimeMillis() - start_time), gyro_yaw));
            }*/
        }
        DbgLog.error("Done with drive forward");
        halt();
    }
    public void PID_move_new(double encoder, double target_heading, double speed) throws InterruptedException
    {
        PID_move_new(encoder,target_heading,speed, false, 12000);
    }
    public void PID_move_new(double encoder, double target_heading, double speed, boolean enableCamera) throws InterruptedException
    {
        PID_move_new(encoder,target_heading,speed, enableCamera, 12000);
    }
    public void PID_move_new(double encoder, double target_heading, double speed, boolean enableCamera, long timeout) throws InterruptedException {
        if(!this.opModeIsActive())
            return;
        DbgLog.error("PID_move begin, target_heading: " + String.format("%.2f, gyro: %.2f",target_heading, getGyroYaw()));
        tele.addData("inside ", "the PID_move_new");
        speed = Range.clip(Math.abs(speed), 0, 1);
        this.target_heading = target_heading;
        //encoder *= 2;   // after I fixed some encoders, I realized that the
        // robot was moving half the intended distance due to an error.
        // this saves me the time of doubling every value from now on. Whoops :/
        //motorFL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //motorFL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //All errors are in terms of heading: the robot's yaw
        double max;
        double gyro_yaw = getGyroYaw();
        double new_yaw;
        resetStartTime();
        double safety_time = System.currentTimeMillis() + timeout;
        double start_time = System.currentTimeMillis();
        if(!this.opModeIsActive())
            return;
        double PID_change = 0;
        double right;
        double left;
        double kP = this.kP;
        double kI = this.kI;
        double kD = this.kD * 1.5;
        motorFL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int currentFLPosition = motorFL.getCurrentPosition();                            //measures current encoder value
        int currentFRPosition = motorFR.getCurrentPosition();                            //measures current encoder value
        int currentBLPosition = motorBL.getCurrentPosition();                            //measures current encoder value
        int currentBRPosition = motorBR.getCurrentPosition();                            //measures current encoder value*/
        int currentEncoder = 0; //=mid2(currentBLPosition, currentBRPosition, currentFLPosition, currentFRPosition);
        if(!this.opModeIsActive())
            return;
        resetPID();
        while(Math.abs(currentEncoder) < Math.abs(encoder) && System.currentTimeMillis() < safety_time && System.currentTimeMillis() < global_timeout && this.opModeIsActive())  //moves until encoders change by value inputted
        {
            tele.addData("driving: ", "forward");
            //DbgLog.error(String.format("i start: %.2f", iError));
            PID_change = get_PID(kP, kI, kD) * 2 * speed; // scales it to be proportional to the speed
            right = speed + PID_change;
            left = speed - PID_change;
            max = Math.max(Math.abs(right), Math.abs(left));
            //This standardizes the speeds, so that they are correct relative to each other,
            //and that one of the two will be equivalent, and neither greater, than "speed"
            right /= max;
            right *= speed;
            left /= max;
            left *= speed;
            /*if(Math.abs(currentEncoder) > 5000)
            {
                right /= 1.8;
                left /= 1.8;
            }*/
            if(encoder < 0) {
                setRightPower(-right);
                setLeftPower(-left);
            }
            else {
                setRightPower(right);
                setLeftPower(left);
            }
            //DEBUG LOGGING
            //DbgLog.error(String.format("i end: %.2f", iError));
            tele.addData("a: ", String.format("k*error: %.2f, k*dError: %.2f, k*iError: %.2f", kP * error, kD * dError, kI * iError));
            tele.addData("b: ", String.format("left: %.2f, right: %.2f", left, right));
            tele.addData(" ", String.format("gyro yaw: %.2f, target: %.2f", gyro_yaw, this.target_heading));
            if(!this.opModeIsActive())
                return;
            DbgLog.error(String.format("k*error: %.2f, k*dError: %.2f, k*iError: %.2f, PID: %.2f", kP * error, kD * dError, kI * iError, PID_change));
            DbgLog.error(String.format("left: %.2f, right: %.2f, gyro: %.2f, target: %.2f", left, right, gyro_yaw, this.target_heading));
            currentEncoder = mid2(motorBL.getCurrentPosition() - currentBLPosition, motorFL.getCurrentPosition() - currentFLPosition, motorBR.getCurrentPosition() - currentBRPosition, motorBR.getCurrentPosition() - currentBRPosition /*motorBR.getCurrentPosition() - 0 currentBRPosition*/);
            DbgLog.error("currentEncoder: " + currentEncoder + ", target_enc: " + String.format("%.2f",encoder));
            DbgLog.error("FR: "+ String.format("%d, FL: %d", motorFR.getCurrentPosition() - currentFRPosition, motorFL.getCurrentPosition() - currentFLPosition));
            DbgLog.error("BR: " + String.format("%d, BL: %d", motorBR.getCurrentPosition() - currentBRPosition, motorBL.getCurrentPosition() - currentBLPosition));
            if(!this.opModeIsActive())
                return;
            //END DEBUG LOGGING

            this.waitOneFullHardwareCycle();
            /*if(enableCamera && beacon.getAnalysisMethod().equals(Beacon.AnalysisMethod.FAST))
            {
                DbgLog.error("Beacon Color" + beacon.getAnalysis().getColorString());
                DbgLog.error("Beacon Confidence" + beacon.getAnalysis().getConfidenceString());
                DbgLog.error("Beacon Location (Center)" + String.format("x: %.2f y: %.2f", beacon.getAnalysis().getCenter().y, beacon.getAnalysis().getCenter().x));
                DbgLog.error("Relative " + String.format("x: %.2f y: %.2f", beacon.getAnalysis().getCenter().y / 480.0, beacon.getAnalysis().getCenter().x / 864.0));
                DbgLog.error("New Heading: " + NewRobotics.calculate_heading(new DPoint(beacon.getAnalysis().getCenter().y, beacon.getAnalysis().getCenter().x)));
                if(beacon.getAnalysis().getConfidence() > 0.2) { // beacon.getAnalysis().getConfidence() > 0.1 && <--- use this if COMPLEX analysis is on
                    double getHeading = -getHeading(System.currentTimeMillis() - start_time);
                    this.target_heading = gyro_yaw + getHeading;
                    DbgLog.error(String.format("new heading target1: %.2f, new2:%.2f, old heading: %.2f", this.target_heading,
                            gyro_yaw - NewRobotics.calculate_heading(new DPoint(beacon.getAnalysis().getCenter().y, beacon.getAnalysis().getCenter().x), (System.currentTimeMillis() - start_time) / Math.pow(10, 3)),
                            gyro_yaw));
                }
            }
            else if(enableCamera && beacon.getAnalysis().isBeaconFound()) { // beacon.getAnalysis().getConfidence() > 0.1 && <--- use this if COMPLEX analysis is on
                this.target_heading = gyro_yaw + getHeading(System.currentTimeMillis() - start_time);
                DbgLog.error(String.format("new heading target: %.2f, old heading: %.2f", gyro_yaw +  getHeading(System.currentTimeMillis() - start_time), gyro_yaw));
            }*/
        }
        if(!this.opModeIsActive())
            return;
        DbgLog.error("Done with drive forward");
        halt();
        waitOneFullHardwareCycle();
        halt();
        halt();
    }
    public double get_PID() throws InterruptedException
    {
        return get_PID(getGyroYaw(), kP, kI, kD);
    }
    public double get_PID(double gyro) throws InterruptedException
    {
        return get_PID(gyro, kP, kI, kD);
    }
    public double get_PID(double kP, double kI, double kD) throws InterruptedException
    {
        return get_PID(getGyroYaw(), kP, kI, kD);
    }
    public double get_PID(double gyro, double kP, double kI, double kD) throws InterruptedException
    {
        if(getRuntime() - time < 0.1) // no new data available
            return prevPID;
        prevGyro = gyro;
        dt = getRuntime() - time;
        time = getRuntime();
        prevError = error;
        if(Math.abs(target_heading - gyro + 360) < Math.abs(target_heading - gyro))
            error = target_heading - gyro + 360;
        else
            error = target_heading - gyro;
        dError = (error - prevError) / dt;
        //make this a reimann right sum if needed to improve speed at the cost of accuracy
        iError = Range.clip(iError + 0.5 * (prevError + error) * dt, -125, 125) * 0.99; // a trapezoidal approximation of the integral.
        prevPID = kP * error + kD * dError + kI * iError;
        return prevPID;
    }
    public double get_PID_Pitch(double target_pitch, double kP, double kI, double kD) throws InterruptedException
    {
        double gyro = getGyroPitch();
        if(!this.opModeIsActive())
            return 0;
        dt = getRuntime() - time;
        time = getRuntime();
        prevError = error;
        if(Math.abs(target_pitch - gyro + 360) < Math.abs(target_pitch - gyro))
            error = target_pitch - gyro + 360;
        else
            error = target_pitch - gyro;
        dError = (error - prevError) / dt;
        //make this a reimann right sum if needed to improve speed at the cost of accuracy
        iError = Range.clip(iError + 0.5 * (prevError + error) * dt, -125, 125) * 0.99; // a trapezoidal approximation of the integral.
        return kP * error + kD * dError + kI * iError;
    }
    public void resetPID() throws InterruptedException
    {
        if(!this.opModeIsActive())
            return;
        if(Math.abs(target_heading - getGyroYaw() + 360) < Math.abs(target_heading - getGyroYaw()))
            error = target_heading - getGyroYaw() + 360;
        else
            error = target_heading - getGyroYaw();
        time = getRuntime();
        iError = 0;
        prevPID = 0;
        prevError = 0;
        prevGyro = 0;
    }
    public void set_target_heading(double target_heading)
    {
        this.target_heading = target_heading;
    }
    public double getHeading() throws InterruptedException
    {
        return getHeading(0);
    }
    public double getHeading(double millis) throws InterruptedException
    {
        if(!this.opModeIsActive())
            return -1;
        return this.calculate_heading(new DPoint(beacon.getAnalysis().getCenter().x, beacon.getAnalysis().getCenter().y), millis / Math.pow(10, 3));
        //return NewRobotics.calculate_heading(DPoint.makeDPoint(beacon.getAnalysis().getCenter()), t / Math.pow(10, 6));
    }
    public double calculate_heading(DPoint center, double time) //encoder represents distance travelled. max_encoder is target distance
    {
        if(time > 3) // it is too close to the beacon by this point
            return 0;
        double ratio = 1.0 / 1.32; // width of view divided by distance to object- tested with ruler
        int width_pixels = 480; // width of screen
        double depth_inches = 18 * (1 - (time / 4)); //18
        double width_inches = depth_inches * ratio; // 13.64
        //double width_deg = Math.atan(ratio) * 2; // field of view of the camera, in degrees
        double center_x = center.x;
        double x_offset_pix = center_x - (width_pixels / 2) + 20; // in pixels
        double x_offset_inches = (x_offset_pix * width_inches / width_pixels); // phone is one inch from center
        double deg_offset = Math.toDegrees(Math.atan(x_offset_inches*2 / depth_inches)); // difference between current heading and target
        DbgLog.error(String.format("head: %.2f, x_off_in: %.2f, depth_in: %.2f, x_off_pix: %.2f",deg_offset, x_offset_inches, depth_inches, x_offset_pix));
        tele.addData("head: ", String.format("%.2f, x_off_in: %.2f, depth_in: %.2f, x_off_pix: %.2f",deg_offset, x_offset_inches, depth_inches, x_offset_pix));
        return Range.clip(deg_offset, -15, 15) / 1.8;
    }
    public double getKP()
    {
        return kP;
    }
    public double getKI()
    {
        return kI;
    }
    public double getKD()
    {
        return kD;
    }
    public void setKP(double kP)
    {
        this.kP = kP;
    }
    public void setKI(double kI)
    {
        this.kI = kI;
    }
    public void setKD(double kD)
    {
        this.kD = kD;
    }
    //===========================================================
    //====================END PID CONTROL========================
    //===========================================================
    double red_left_cnt = 0;
    double blue_left_cnt = 0;
    public double find_Beacon() throws InterruptedException
    {
        if(!this.opModeIsActive())
            return -1;
        resetStartTime();
        double total_heading = 0;
        double cnt = 0;
        double gyro_yaw;
        discardFrame();
        while(this.opModeIsActive() && getRuntime() < 5)
        {
            if(this.hasNewFrame())
            {
                gyro_yaw = getGyroYaw();
                DbgLog.error("finding beacon...");
                if(beacon.getAnalysis().getConfidence() > 0.3) { // beacon.getAnalysis().getConfidence() > 0.1 && <--- use this if COMPLEX analysis is on
                    if(!this.opModeIsActive())
                        return -1;
                    tele.addData("beacon: ", String.format("change: %.2f", getHeading()));
                    DbgLog.error("Beacon Color" + beacon.getAnalysis().getColorString());
                    DbgLog.error("Beacon Location (Center)" + beacon.getAnalysis().getLocationString());
                    DbgLog.error("Beacon Confidence" + beacon.getAnalysis().getConfidenceString());
                    //DbgLog.error("New Heading: " + NewRobotics.calculate_heading(new DPoint(beacon.getAnalysis().getCenter().y, beacon.getAnalysis().getCenter().x)) + String.format(" old: %.2f", getGyroYaw()));
                    DbgLog.error("Relative " + String.format("y: %.2f x: %.2f", beacon.getAnalysis().getCenter().y / 480.0, beacon.getAnalysis().getCenter().x / 864.0));
                    //DbgLog.error("New Heading" + NewRobotics.calculate_heading(new DPoint(beacon.getAnalysis().getCenter().y, beacon.getAnalysis().getCenter().x)));
                    DbgLog.error(String.format("new heading target xy: %.2f, new yx: %.2f, old heading: %.2f", gyro_yaw +  getHeading(), 0.0, gyro_yaw));
                    if(!this.opModeIsActive())
                        return -1;
                    total_heading += getHeading(0);
                    if(!this.opModeIsActive())
                        return -1;
                    cnt++;
                    if(beacon.getAnalysis().isLeftKnown() && beacon.getAnalysis().isLeftRed())
                    {
                        red_left_cnt++;
                    }
                    else if(beacon.getAnalysis().isLeftKnown() && beacon.getAnalysis().isLeftBlue())
                    {
                        blue_left_cnt++;
                    }
                }
                this.waitOneFullHardwareCycle();
                //halt();
                discardFrame();
            }
            this.waitOneFullHardwareCycle();
        }
        DbgLog.error("cnt: " + cnt);
        if(cnt > 0)
            return (total_heading / cnt);
        else
            return 0;
    }

    //x=.36
    //x=.56

    double makePositive(double heading_Deg)
    {
        return heading_Deg < 0? heading_Deg + 360: heading_Deg;
    }
    void setRightPower(double power)
    {
        if(!this.opModeIsActive())
            return;
        motorBR.setPower(Range.clip(power, -1, 1));
        motorFR.setPower(Range.clip(power, -1, 1));
    }
    void setRightPower(double powerF, double powerB)
    {
        if(!this.opModeIsActive())
            return;
        motorBR.setPower(Range.clip(powerB, -1, 1));
        motorFR.setPower(Range.clip(powerF, -1, 1));
    }
    void setLeftPower(double power)
    {
        if(!this.opModeIsActive())
            return;
        motorBL.setPower(Range.clip(-power, -1, 1));
        motorFL.setPower(Range.clip(-power, -1, 1));
    }
    void setLeftPower(double powerF, double powerB)
    {
        if(!this.opModeIsActive())
            return;
        motorBL.setPower(Range.clip(-powerB, -1, 1));
        motorFL.setPower(Range.clip(-powerF, -1, 1));
    }

    //==================BLUE INITIAL SEGMENTS==================
    //These are all in terms of squares moved
    //E.g. move(-2,1) means to move two squares backwards


    public void Close_Blue_Buttons2() throws InterruptedException {
        move2(0.75);// , 1);
        tele.addData("we made it: ", "yay");
        //halt();
        //ssleep(1000);
        turn(-14, 0.75);//turn(-23, 1);
        //encoderTurn(45, 1);
        move2(4.25 * (Math.sqrt(2)));// , 0.75);
        turn(-18, 0.75);
        //encoderTurn(-45, 1);
        move2(0.25);// , 0.5);
        halt();
    }
    double long_encoderB = 1.0 * Math.sqrt(2);
    public void Close_Blue_Buttons_CV() throws InterruptedException {
        PID_move(squares_to_Encoder(3), 0, 1, false);// , 1);
        tele.addData("we made it: ", "yay");
        //halt();
        //ssleep(1000);

        turn(-30, 0.2);//turn(-23, 1);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        //encoderTurn(45, 1);
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        double heading = getGyroYaw();
        PID_move(squares_to_Encoder(8 * Math.sqrt(2)), heading, 1, false);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        /*PID_move(squares_to_Encoder(1 * (Math.sqrt(2))), heading, 1, true);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 2) // wait 2 seconds*/
        turn(-12, 0.2);
        tele.addData(" ", String.format("gyro yaw: %.2f", getGyroYaw()));
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        //encoderTurn(-45, 1);
        DbgLog.error(String.format("%.2f", getGyroYaw()));
        PID_move(squares_to_Encoder(2.5), getGyroYaw(), 1, true, 4000);// , 0.5);
        resetStartTime();
        halt();
        dump_Climbers();
        halt();
    }
    public void Close_Blue_Buttons_CV_new_PID() throws InterruptedException {
        PID_move_new(squares_to_Encoder(3.1 * 57.0 / 72), 0, 1, false);// , 1);
        if(!this.opModeIsActive())
            return;
        tele.addData("we made it: ", "yay");
        //halt();
        //ssleep(1000);
        turn(-54, 0.04);//turn(-23, 1);
        if(!this.opModeIsActive())
            return;
        //encoderTurn(45, 1);
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        double heading = getGyroYaw();
        if(!this.opModeIsActive())
            return;
        PID_move_new(squares_to_Encoder(7.5 * 57.0 / 72 * Math.sqrt(2)), heading, 1, false);
        if(!this.opModeIsActive())
            return;
        /*PID_move(squares_to_Encoder(1 * (Math.sqrt(2))), heading, 1, true);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 2) // wait 2 seconds*/
        turn(-38, 0.04);
        if(!this.opModeIsActive())
            return;
        DbgLog.error(String.format("%.2f", getGyroYaw()));
        double head = getGyroYaw() + find_Beacon();
        if(!this.opModeIsActive())
            return;
        DbgLog.error("you about to be movin, heading: " + String.format("%.2f", head));
        target_heading = head;
        tele.addData("head: ", head);
        PID_move_new(squares_to_Encoder(5.5 * 57.0 / 72), head, 1, true, 1700);// , 0.5);
        if(!this.opModeIsActive())
            return;
        halt();
        dump_Climbers();
        if(!this.opModeIsActive())
            return;
        halt();
        reverse_blue();
        if(!(blue_left_cnt > red_left_cnt))
        {
            push_Right();
        }
        else if(!(red_left_cnt > blue_left_cnt)) // if they are equal, neither will run
        {
            push_Left();
        }
    }
    public void Worlds_Align_Beacon_Blue() throws InterruptedException
    {
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        PID_move_new(squares_to_Encoder(-3.4 * 57.0 /72), 0, 1, false);// , 1);
        halt();
        if(!this.opModeIsActive())
            return;
        tele.addData("we made it: ", "yay");
        PID_turn_time(-41, 4);//turn(-23, 1);

        if(!this.opModeIsActive())
            return;
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        double heading = getGyroYaw();
        if(!this.opModeIsActive())
            return;
        PID_move_new(squares_to_Encoder(-8.35 * 57.0 / 72 * Math.sqrt(2)), heading, 1, false);
        if(!this.opModeIsActive())
            return;
        PID_turn_time(-42, 4);
        if(!this.opModeIsActive())
            return;
        DbgLog.error(String.format("%.2f", getGyroYaw()));
        adjust_heading();
    }
    public void adjust_heading() throws InterruptedException
    {
        double new_heading = find_Beacon();
        PID_turn_time(new_heading, 4);
    }
    public void Worlds_Blue_Buttons() throws InterruptedException
    {
        if(!this.opModeIsActive())
            return;
        if((blue_left_cnt > red_left_cnt))
        {
            DbgLog.error("target is: left");
            servoButtPush.setPosition(0);

        }
        else if((red_left_cnt > blue_left_cnt))
        {
            servoButtPush.setPosition(1);
            DbgLog.error("target is: right(");
        }
        while (Math.abs(servoButtPush.getPosition() - 0.5) < 0.4) {
            this.waitOneFullHardwareCycle();
        }
        halt();
    }
    public void Worlds_Climbers() throws InterruptedException
    {
        PID_move_new(squares_to_Encoder(-5.5 * 57.0 / 72), target_heading, 1, true, 1700);// , 0.5);
        if(!this.opModeIsActive())
            return;
        halt();
        dump_Climbers();
        if(!this.opModeIsActive())
            return;
        halt();
    }
    public void Worlds_Align_Beacon_Red() throws InterruptedException
    {
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        PID_move_new(squares_to_Encoder(-3.4 * 57.0 / 72), 0, 1, false);// , 1);
        halt();
        if(!this.opModeIsActive())
            return;
        tele.addData("we made it: ", "yay");
        PID_turn_time(41, 4);//turn(-23, 1);

        if(!this.opModeIsActive())
            return;
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        double heading = getGyroYaw();
        if(!this.opModeIsActive())
            return;
        PID_move_new(squares_to_Encoder(-8.35 * 57.0 / 72 * Math.sqrt(2)), heading, 1, false);
        if(!this.opModeIsActive())
            return;
        PID_turn_time(42, 4);
        if(!this.opModeIsActive())
            return;
        DbgLog.error(String.format("%.2f", getGyroYaw()));
        adjust_heading();
    }
    public void Worlds_Red_Buttons() throws InterruptedException
    {
        if(!this.opModeIsActive())
            return;
        if((red_left_cnt > blue_left_cnt))
        {
            DbgLog.error("target is: left");
            servoButtPush.setPosition(0);

        }
        else if((blue_left_cnt > red_left_cnt))
        {
            servoButtPush.setPosition(1);
            DbgLog.error("target is: right(");
        }
        while (Math.abs(servoButtPush.getPosition() - 0.5) < 0.4) {
            this.waitOneFullHardwareCycle();
        }
        halt();
    }
    public void Close_Blue_Buttons_CV_new_PID_No_Dump() throws InterruptedException {
        PID_move_new(squares_to_Encoder(3 * 57.0 /72), 0, 1, false);// , 1);
        if(!this.opModeIsActive())
            return;
        tele.addData("we made it: ", "yay");
        //halt();
        //ssleep(1000);
        turn(-50, 0.04);//turn(-23, 1);
        if(!this.opModeIsActive())
            return;
        //encoderTurn(45, 1);
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        double heading = getGyroYaw();
        if(!this.opModeIsActive())
            return;
        PID_move_new(squares_to_Encoder(6.8 * 57.0 /72 * Math.sqrt(2)), heading, 1, false);
        if(!this.opModeIsActive())
            return;
        /*PID_move(squares_to_Encoder(1 * (Math.sqrt(2))), heading, 1, true);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 2) // wait 2 seconds*/
        turn(-39, 0.04);
        if(!this.opModeIsActive())
            return;
        DbgLog.error(String.format("%.2f", getGyroYaw()));
        double head = getGyroYaw() + find_Beacon();
        if(!this.opModeIsActive())
            return;
        DbgLog.error("you about to be movin, heading: " + String.format("%.2f", head));
        target_heading = head;
        tele.addData("head:", head);
        PID_move_new(squares_to_Encoder(5.5 * 57.0 /72), head, 1, true, 3000);// , 0.5);
        if(!this.opModeIsActive())
            return;
        halt();
        //dump_Climbers();
        if(!this.opModeIsActive())
            return;
        halt();
        if(blue_left_cnt > red_left_cnt)
        {
            push_Right();
        }
        else if(red_left_cnt > blue_left_cnt) // if they are equal, neither will run
        {
            push_Left();
        }
    }
    public void Close_Blue_Buttons_Polar() throws InterruptedException {
        double speed = 0.6;
        PID_move_displacement_polar(squares_to_Encoder(0.75), getGyroYaw(), speed);// , 1);
        tele.addData("we made it: ", "yay");
        //halt();
        //ssleep(1000);
        turn(-22, speed - 0.2);//turn(-23, 1);
        //PID_turn(-22, 2);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        //encoderTurn(45, 1);
        double heading = getGyroYaw();
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        PID_move_displacement_cartesian(1500, -1800, speed, 10);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        //turn(-20, speed - 0.2);
        PID_turn(-20, 2);
        tele.addData(" ", String.format("gyro yaw: %.2f", getGyroYaw()));
        tele.addData("Before the move into the beacon", " ");
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 8) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        double curr_heading = getGyroYaw();
        DbgLog.error(String.format("curr: %.2f", curr_heading));
        PID_move(squares_to_Encoder(2.5), curr_heading, speed, true, 3000);// , 0.5);
        resetStartTime();
        halt();
        dump_Climbers();
        halt();
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 5) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        if(beacon.getAnalysis().isLeftKnown() && beacon.getAnalysis().isLeftBlue())
        {
            push_Right();
        }
        else if(beacon.getAnalysis().isRightKnown() && beacon.getAnalysis().isRightBlue())
        {
            push_Left();
        }
    }
    public void Close_Red_Buttons_CV_new_PID() throws InterruptedException {
        tele.addData("before PID", " done");
        PID_move_new(squares_to_Encoder(3.1 * 57.0 / 72), 0, 1, false);// , 1);
        if(!this.opModeIsActive())
            return;
        tele.addData("we made it: ", "yay");
        //halt();
        //ssleep(1000);
        turn(46, 0.03);//turn(-23, 1);
        if(!this.opModeIsActive())
            return;
        //encoderTurn(45, 1);
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        double heading = getGyroYaw();
        if(!this.opModeIsActive())
            return;
        PID_move_new(squares_to_Encoder(7 * Math.sqrt(2) * 57.0 /72), heading, 1, false);
        if(!this.opModeIsActive())
            return;
        /*PID_move(squares_to_Encoder(1 * (Math.sqrt(2))), heading, 1, true);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 2) // wait 2 seconds*/
        tele.addData("pre: ", String.format("gyro yaw: %.2f", getGyroYaw()));
        turn(38, 0.03);
        if(!this.opModeIsActive())
            return;
        tele.addData("post: ", String.format("gyro yaw: %.2f", getGyroYaw()));
        //encoderTurn(-45, 1);
        DbgLog.error(String.format("%.2f", getGyroYaw()));
        // .31, .38: 9.9

        double head = getGyroYaw() + find_Beacon();
        tele.addData("head: ", head);
        if(!this.opModeIsActive())
            return;
        DbgLog.error("you about to be movin, heading: " + String.format("%.2f", head));
        target_heading = head;
        PID_move_new(squares_to_Encoder(5.5 * 57.0 / 72), head, 1, true, 1700);// , 0.5);
        if(!this.opModeIsActive())
            return;
        halt();
        dump_Climbers();
        if(!this.opModeIsActive())
            return;
        halt();
        reverse_red();
        if(blue_left_cnt > red_left_cnt)
        {
            push_Left();
        }
        else if(red_left_cnt > blue_left_cnt)
        {
            push_Right();
        }
    }
    public void Close_Red_Buttons_CV_New_PID_Worlds() throws InterruptedException {
        tele.addData("before PID", " done");
        PID_move_new(squares_to_Encoder(-3.1 * 57.0 /72), 0, 1, false);// , 1);
        if(!this.opModeIsActive())
            return;
        tele.addData("we made it: ", "yay");
        //halt();
        //ssleep(1000);
        turn(-46, 0.03);//turn(-23, 1);
        if(!this.opModeIsActive())
            return;
        //encoderTurn(45, 1);
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        double heading = getGyroYaw();
        if(!this.opModeIsActive())
            return;
        PID_move_new(squares_to_Encoder(-7 * Math.sqrt(2) * 57.0 /72), heading, 1, false);
        if(!this.opModeIsActive())
            return;
        /*PID_move(squares_to_Encoder(1 * (Math.sqrt(2))), heading, 1, true);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 2) // wait 2 seconds*/
        tele.addData("pre: ", String.format("gyro yaw: %.2f", getGyroYaw()));
        turn(-38, 0.03);
        if(!this.opModeIsActive())
            return;
        tele.addData("post: ", String.format("gyro yaw: %.2f", getGyroYaw()));
        //encoderTurn(-45, 1);
        DbgLog.error(String.format("%.2f", getGyroYaw()));
        // .31, .38: 9.9

        double head = getGyroYaw() + find_Beacon();
        tele.addData("head: ", head);
        if(!this.opModeIsActive())
            return;
        DbgLog.error("you about to be movin, heading: " + String.format("%.2f", head));
        target_heading = head;
        PID_move_new(squares_to_Encoder(-5.5 * 57.0 / 72), head, 1, true, 1700);// , 0.5);
        if(!this.opModeIsActive())
            return;
        halt();
        dump_Climbers();
        if(!this.opModeIsActive())
            return;
        halt();
        reverse_red();
        if((blue_left_cnt > red_left_cnt))
        {
            servoButtPush.setPosition(1);
        }
        else if((red_left_cnt > blue_left_cnt)) // if they are equal, neither will run
        {
            servoButtPush.setPosition(0);
        }
        while (Math.abs(servoButtPush.getPosition() - 0.5) < 0.4) {
            this.waitOneFullHardwareCycle();
        }
        PID_move(squares_to_Encoder(-1.3), getGyroYaw(), 0.5, false, 3000);
        halt();
    }
    public void Close_Red_Buttons_CV_new_PID_No_Dump() throws InterruptedException {
        PID_move_new(squares_to_Encoder(3 * 57.0 /72), 0, 1, false);// , 1);
        if(!this.opModeIsActive())
            return;
        tele.addData("we made it: ", "yay");
        //halt();
        //ssleep(1000);
        turn(46, 0.04);//turn(-23, 1);
        if(!this.opModeIsActive())
            return;
        //encoderTurn(45, 1);
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        double heading = getGyroYaw();
        if(!this.opModeIsActive())
            return;
        PID_move_new(squares_to_Encoder(6.8 * 57.0 /72 * Math.sqrt(2)), heading, 1, false);
        if(!this.opModeIsActive())
            return;
        /*PID_move(squares_to_Encoder(1 * (Math.sqrt(2))), heading, 1, true);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 2) // wait 2 seconds*/
        tele.addData("pre: ", String.format("gyro yaw: %.2f", getGyroYaw()));
        turn(38, 0.04);
        if(!this.opModeIsActive())
            return;
        tele.addData("post: ", String.format("gyro yaw: %.2f", getGyroYaw()));
        //encoderTurn(-45, 1);
        DbgLog.error(String.format("%.2f", getGyroYaw()));
        // .31, .38: 9.9

        double head = getGyroYaw() + find_Beacon();
        if(!this.opModeIsActive())
            return;
        DbgLog.error("you about to be movin, heading: " + String.format("%.2f", head));
        target_heading = head;
        PID_move_new(squares_to_Encoder(5.5 * 57.0 /72), head, 1, true, 3000);// , 0.5);
        if(!this.opModeIsActive())
            return;
        halt();
        //dump_Climbers();
        if(!this.opModeIsActive())
            return;
        halt();
        if(blue_left_cnt > red_left_cnt)
        {
            push_Left();
        }
        else if(red_left_cnt > blue_left_cnt)
        {
            push_Right();
        }
    }
    public void Close_Blue_Buttons_Cartesian() throws InterruptedException {
        double speed = 0.6;
        PID_move_displacement_polar(squares_to_Encoder(0.75), getGyroYaw(), speed);// , 1);
        tele.addData("we made it: ", "yay");
        //halt();
        //ssleep(1000);
        turn(-22, speed - 0.2);//turn(-23, 1);
        //PID_turn(-22, 2);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        //encoderTurn(45, 1);
        double heading = getGyroYaw();
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        PID_move_displacement_polar(squares_to_Encoder(long_encoderB + 1 * Math.sqrt(2)), heading, speed);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        turn(-20, speed - 0.2);
        tele.addData(" ", String.format("gyro yaw: %.2f", getGyroYaw()));
        tele.addData("Before the move into the beacon", " ");
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 8) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        DbgLog.error(String.format("%.2f", getGyroYaw()));
        PID_move(squares_to_Encoder(2.5), getGyroYaw(), speed, true, 2500);// , 0.5);
        resetStartTime();
        halt();
        dump_Climbers();
        halt(); halt(); halt();
        reverse_red();
        if(beacon.getAnalysis().isLeftKnown() && beacon.getAnalysis().isLeftBlue())
        {
            push_Right();
        }
        else if(beacon.getAnalysis().isRightKnown() && beacon.getAnalysis().isRightBlue())
        {
            push_Left();
        }
    }
    public void reverse_red() throws InterruptedException
    {
        if(!this.opModeIsActive())
            return;
        double currentEncoder = 0;
        int currentFLPosition = motorFL.getCurrentPosition();                            //measures current encoder value
        int currentFRPosition = motorFR.getCurrentPosition();                            //measures current encoder value
        int currentBLPosition = motorBL.getCurrentPosition();                            //measures current encoder value
        int currentBRPosition = motorBR.getCurrentPosition();                            //measures current encoder value*/
        resetStartTime();
        while(Math.abs(currentEncoder) < Math.abs(1100) && this.opModeIsActive() && System.currentTimeMillis() < global_timeout && getRuntime() < 1)
        {
            setLeftPower(-.8);
            setRightPower(-.8);
            currentEncoder = mid2(motorBL.getCurrentPosition() - currentBLPosition, motorFL.getCurrentPosition() - currentFLPosition, motorBR.getCurrentPosition() - currentBRPosition, motorBR.getCurrentPosition() - currentBRPosition /*motorBR.getCurrentPosition() - 0 currentBRPosition*/);
        }
        halt();
        if(!this.opModeIsActive())
            return;
        halt();
        if(!this.opModeIsActive())
            return;
        turn(-165, 0.2);
        if(!this.opModeIsActive())
            return;
        resetStartTime();
        currentEncoder = 0;
        currentFLPosition = motorFL.getCurrentPosition();                            //measures current encoder value
        currentFRPosition = motorFR.getCurrentPosition();                            //measures current encoder value
        currentBLPosition = motorBL.getCurrentPosition();                            //measures current encoder value
        currentBRPosition = motorBR.getCurrentPosition();                            //measures current encoder value*/
        while(Math.abs(currentEncoder) < Math.abs(200) && this.opModeIsActive() && System.currentTimeMillis() < global_timeout && getRuntime() < 0.4)
        {
            setLeftPower(-.8);
            setRightPower(-.8);
            currentEncoder = mid2(motorBL.getCurrentPosition() - currentBLPosition, motorFL.getCurrentPosition() - currentFLPosition, motorBR.getCurrentPosition() - currentBRPosition, motorBR.getCurrentPosition() - currentBRPosition /*motorBR.getCurrentPosition() - 0 currentBRPosition*/);
        }
        if(!this.opModeIsActive())
            return;
        halt();
        resetStartTime();
        while(getRuntime() < 1.5)
            this.waitOneFullHardwareCycle();
    }
    public void reverse_blue() throws InterruptedException
    {
        double speed = -0.8;
        if(!this.opModeIsActive())
            return;
        double currentEncoder = 0;
        int currentFLPosition = motorFL.getCurrentPosition();                            //measures current encoder value
        int currentFRPosition = motorFR.getCurrentPosition();                            //measures current encoder value
        int currentBLPosition = motorBL.getCurrentPosition();                            //measures current encoder value
        int currentBRPosition = motorBR.getCurrentPosition();                            //measures current encoder value*/
        resetStartTime();
        while(Math.abs(currentEncoder) < Math.abs(1500) && this.opModeIsActive() && System.currentTimeMillis() < global_timeout && getRuntime() < 1.3)
        {
            DbgLog.error("current Encoder: " + currentEncoder);
            setLeftPower(speed, speed / 3.0);
            setRightPower(speed, speed / 3.0);
            currentEncoder = mid2(motorBL.getCurrentPosition() - currentBLPosition, motorFL.getCurrentPosition() - currentFLPosition, motorBR.getCurrentPosition() - currentBRPosition, motorBR.getCurrentPosition() - currentBRPosition /*motorBR.getCurrentPosition() - 0 currentBRPosition*/);
        }
        halt();
        if(!this.opModeIsActive())
            return;
        halt();
        if(!this.opModeIsActive())
            return;
        timed_turn(155, 0.19, 5, 5);
        if(!this.opModeIsActive())
            return;
        currentFLPosition = motorFL.getCurrentPosition();                            //measures current encoder value
        currentFRPosition = motorFR.getCurrentPosition();                            //measures current encoder value
        currentBLPosition = motorBL.getCurrentPosition();                            //measures current encoder value
        currentBRPosition = motorBR.getCurrentPosition();                            //measures current encoder value*/
        resetStartTime();
        DbgLog.error("current Encoder: " + currentEncoder);
        currentEncoder = 0;
        while(Math.abs(currentEncoder) < Math.abs(500) && this.opModeIsActive() && System.currentTimeMillis() < global_timeout && getRuntime() < .4)
        {
            DbgLog.error("current Encoder: " + currentEncoder);
            setLeftPower(speed, speed / 3.0);
            setRightPower(speed, speed / 3.0);
            currentEncoder = mid2(motorBL.getCurrentPosition() - currentBLPosition, motorFL.getCurrentPosition() - currentFLPosition, motorBR.getCurrentPosition() - currentBRPosition, motorBR.getCurrentPosition() - currentBRPosition /*motorBR.getCurrentPosition() - 0 currentBRPosition*/);
        }
        if(!this.opModeIsActive())
            return;
        halt();
        resetStartTime();
        while(getRuntime() < 1.5)
            this.waitOneFullHardwareCycle();
    }
    public void push_Left() throws InterruptedException
    {
        if(!this.opModeIsActive())
            return;
        resetStartTime();
        servoButtonL.setPosition(0);
        while (this.opModeIsActive() && getRuntime() < 5)
        {
            this.waitOneFullHardwareCycle();
        }
        if(!this.opModeIsActive())
            return;
        resetStartTime();
        servoButtonL.setPosition(1);
        while (this.opModeIsActive() && getRuntime() < 5)
        {
            this.waitOneFullHardwareCycle();
        }
        if(!this.opModeIsActive())
            return;
        servoButtonL.setPosition(0.5);
    }

    public void push_Right() throws InterruptedException
    {
        if(!this.opModeIsActive())
            return;
        resetStartTime();
        servoButtonR.setPosition(0);
        while (this.opModeIsActive() && getRuntime() < 5)
        {
            this.waitOneFullHardwareCycle();
        }
        if(!this.opModeIsActive())
            return;
        servoButtonR.setPosition(1);
        resetStartTime();
        while (this.opModeIsActive() && getRuntime() < 5)
        {
            this.waitOneFullHardwareCycle();
        }
        if(!this.opModeIsActive())
            return;
        servoButtonR.setPosition(0.5);
    }

    public void Close_Blue_Buttons_CV_NoDump() throws InterruptedException {
        PID_move(squares_to_Encoder(0.75), 0, 1, false);// , 1);
        tele.addData("we made it: ", "yay");
        //halt();
        //ssleep(1000);

        turn(-32.5, 0.75);//turn(-23, 1);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        //encoderTurn(45, 1);
        double heading = getGyroYaw();
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        PID_move(squares_to_Encoder(long_encoderB + 1 * Math.sqrt(2)), heading, 1, false);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        /*PID_move(squares_to_Encoder(1 * (Math.sqrt(2))), heading, 1, true);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 2) // wait 2 seconds*/
        turn(-59, 0.75);
        tele.addData(" ", String.format("gyro yaw: %.2f", getGyroYaw()));
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        //encoderTurn(-45, 1);
        DbgLog.error(String.format("%.2f", getGyroYaw()));
        PID_move(squares_to_Encoder(2.5), getGyroYaw(), 1, true, 4000);// , 0.5);
        resetStartTime();
        halt();
    }
    double long_encoderR = 1.23 * Math.sqrt(2);
    public void Close_Red_Buttons_CV() throws InterruptedException {
        PID_move(squares_to_Encoder(0.75), 0, 1, false);// , 1);
        tele.addData("we made it: ", "yay");
        //halt();
        //ssleep(1000);

        turn(32.5, 0.75);//turn(-23, 1);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        //encoderTurn(45, 1);
        double heading = getGyroYaw();
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        PID_move(squares_to_Encoder(long_encoderR + 1 * Math.sqrt(2)), heading, 1, false);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        /*PID_move(squares_to_Encoder(1 * (Math.sqrt(2))), heading, 1, true);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 2) // wait 2 seconds*/
        turn(46, 0.75);
        tele.addData(" ", String.format("gyro yaw: %.2f", getGyroYaw()));
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        //encoderTurn(-45, 1);
        DbgLog.error(String.format("%.2f", getGyroYaw()));
        PID_move(squares_to_Encoder(2.5), getGyroYaw(), 1, true, 4000);// , 0.5);
        resetStartTime();
        halt();
        dump_Climbers();
        halt();
    }

    public void Close_Red_Buttons_CV_NoDump() throws InterruptedException {
        PID_move(squares_to_Encoder(0.75), 0, 1, false);// , 1);
        tele.addData("we made it: ", "yay");
        //halt();
        //ssleep(1000);
        turn(32.5, 0.75);//turn(-23, 1);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        //encoderTurn(45, 1);
        double heading = getGyroYaw();
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        PID_move(squares_to_Encoder(long_encoderR + 1 * Math.sqrt(2)), heading, 1, false);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        /*PID_move(squares_to_Encoder(1 * (Math.sqrt(2))), heading, 1, true);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 2) // wait 2 seconds*/
        turn(59, 0.75);
        tele.addData(" ", String.format("gyro yaw: %.2f", getGyroYaw()));
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        //encoderTurn(-45, 1);
        DbgLog.error(String.format("%.2f", getGyroYaw()));
        PID_move(squares_to_Encoder(2.5), getGyroYaw(), 1, true, 4000);// , 0.5);
        resetStartTime();
        halt();
        dump_Climbers();
        halt();
    }

    public void dump_Climbers() throws InterruptedException
    {
        this.waitOneFullHardwareCycle();
        servoClimberArm.setPosition(1);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 3.5) // wait 2 seconds
            this.waitOneFullHardwareCycle();
        if(!this.opModeIsActive())
            return;
        servoClimberArm.setPosition(0.5);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1) // wait 1 seconds
            this.waitOneFullHardwareCycle();
    }
    public void Far_Blue_Buttons() throws InterruptedException {
        move(-1, 1);
        turn(-45, 1);
        //encoderTurn(-45, 1);
        move(-2 * (Math.sqrt(2)), 1);
        turn(-45, 1);
        //encoderTurn(-45, 1);
        move(-2, 1);
    }
    public void Close_Blue_BlueRamp() throws InterruptedException {
        move( 1, 1);
        turn(45, 1);
        //encoderTurn(45, 1);
        move(-Math.sqrt(8) , 1);
        turn(90, 1);
        //encoderTurn(90, 1);
    }
    public void Far_Blue_BlueRamp() throws InterruptedException {
        move(2 , 1);
        turn(90, 1);
        //encoderTurn(90, 1);
        move(3 , 1);
        turn(-45, 1);
        //encoderTurn(-45, 1);
        move(.5 * Math.sqrt(2) , 1);
    }
    public void Close_Blue_RedRamp() throws InterruptedException {
        move(3 , 1);
        turn(-45, 1);
        //encoderTurn(-45, 1);
        move(2 * Math.sqrt(2) , 1);
    }
    public void Far_Blue_RedRamp() throws InterruptedException {
        move(3.5, 1);
        turn(-45, 1);
        //encoderTurn(-45, 1);
    }

//RAMP SEGMENTS
    public void Regionals_Straight() throws InterruptedException
    {
        resetStartTime();
        /*while(this.opModeIsActive() && getRuntime() < 15)
            waitOneFullHardwareCycle();*/
        halt();
        PID_move(squares_to_Encoder(1.75 * Math.sqrt(2)), 0, 1, false);// , 1);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1)
            this.waitOneFullHardwareCycle();
        PID_move(squares_to_Encoder(-0.75 * Math.sqrt(2)), 0, 1, false);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 1)
            this.waitOneFullHardwareCycle();
        resetStartTime();
        turn(-70, 0.75);
        while(this.opModeIsActive() && getRuntime() < 1)
            this.waitOneFullHardwareCycle();
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 2) {
            this.waitOneFullHardwareCycle();
            setLeftPower(-1);
            setRightPower(-1);
        }
        halt();

    }
    public void Regionals_Red_Side_Ramp() throws InterruptedException
    {
        halt();
        resetStartTime();
        /*while(this.opModeIsActive() && getRuntime() < 17)
            waitOneFullHardwareCycle();*/
        PID_move(squares_to_Encoder(0.75), 0, 1, false);// , 1);
        tele.addData("we made it: ", "yay");
        //halt();
        //ssleep(1000);

        turn(32.5, 0.75);//turn(-23, 1);
        resetStartTime();
        while(this.opModeIsActive() && getRuntime() < 0.5) // wait 0.5 seconds
            this.waitOneFullHardwareCycle();
        //encoderTurn(45, 1);
        double heading = getGyroYaw();
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        PID_move(squares_to_Encoder(0.85), heading, 1, false);
        turn(78, 0.75);
        PID_move(squares_to_Encoder(1.75), getGyroYaw(), 1, false, 3000);// , 1);
        halt();
    }

    public void ClearRamp() throws InterruptedException {
        turn(-90, 1);
        //encoderTurn(-90, 1);
        move(.75 , 1);
        move(-1.5 ,1);
        move(.75 ,1);
        turn(90, 1);
        //encoderTurn(90, 1);
    }

    public void ClimbRamp() throws InterruptedException {
        int counter = 0;
        int badData = 0;
        while(ultra.getUltrasonicLevel() < 40) {
            //SOS();
            motorBL.setPower(1);
            //   motorFL.setPower(1);
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

    public void Buttons() throws InterruptedException {
        //colory stuf
    }

//BUTTONS TO RAMP SEGMENTS

    public void RedButtons_RedRamp() throws InterruptedException {
        move( 1, 1);
        turn(90, 1);
        //encoderTurn(90, 1);
        move(1.5 , 1);
        turn(45,1);
        //encoderTurn(45, 1);
    }
    public void RedButtons_BlueRamp() throws InterruptedException {
        move(4 , 1);
        turn(-45,1);
        //encoderTurn(-45, 1);
    }
    public void BlueButtons_RedRamp() throws InterruptedException {
        move(-1, 1);
        turn(90, 1);
        //encoderTurn(90, 1);
        move(1.5 , 1);
        turn(-45,1);
        //encoderTurn(45, 1);
    }
    public void BlueButtons_BlueRamp() throws InterruptedException {
        move(4 , 1);
        turn(-45,1);
        //encoderTurn(-45, 1);
    }



    public void IMUFinder() throws InterruptedException{
        long oldTime = 0;
        long currTime = 0;
        double oldX = 0;
        double normX = 0;
        double rotX = 0;
        double oldY = 0;
        double normY = 0;
        double rotY = 0;
        double oldZ = 0;
        double normZ = 0;
        double xVel = 0;
        double yVel = 0;
        while(true)
        {
            oldTime = currTime;
            currTime = System.currentTimeMillis();          //x' = x cos f - y sin f       y' = y cos f + x sin f
            long dt = currTime - oldTime;
            oldX = rotX;
            oldY = rotY;
            //normX = ;
            //normY = ;
            //normZ = ;
            rotX = normX * Math.cos(Math.toRadians(getGyroYaw())) - normY * Math.sin(Math.toRadians(getGyroYaw()));
            rotY = normY * Math.cos(Math.toRadians(getGyroYaw())) + normX * Math.sin(Math.toRadians(getGyroYaw()));
            xVel = (rotX + oldX) * .5 * dt;
            yVel = (rotY + oldY) * .5 * dt;
        }
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
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

import java.util.concurrent.TimeUnit;

/**
 * Created by Venom6209 on 10/5/2015.
 */
public class AutonomousSegments extends LinearOpModeCV {

    UltrasonicSensor ultra;
    UltrasonicSensor frontUltra;
    //protected AdafruitIMU IMU;
    protected AdafruitIMU IMU2;
    Telemetry tele;
    LinearOpModeCV parent_op;

    double cm_rotation = 4*Math.PI*2.54;
    double square_per_rot = 60.0/cm_rotation;                  //different units used for measuring distance moved
    double inches = 1.5*Math.PI;
    double degrees = 2000.0/90.0;
    double xPos = 0;
    double yPos = 0;

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
            parent_op.waitOneFullHardwareCycle();
            //waitOneFullHardwareCycle();
        }
        tele.addData("made it to the end of loop", " done");
        halt();
    }
    public static int mid2(int enc1, int enc2, int enc3, int enc4)
    {
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
        //Range.clip(Math.abs(speed), -1, 1);
        double gyro = getGyroYaw();
        deg += gyro;
        deg %= 180;
        //deg now between -180 and 180
        String format;
        DbgLog.error(String.format("start: deg:%.2f, gyro:%.2f", deg, getGyroYaw()));
        if(gyro < deg)
        {
            while(gyro < deg && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout)
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
            /*while(gyro < deg && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout)
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
            while(gyro > deg + 3 && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout)
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
            /*while(gyro > deg && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout)
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
        DbgLog.error(String.format("end: deg:%.2f, gyro:%.2f",deg, gyro));
        this.target_heading = gyro;
        //if(Math.abs(getGyroYaw() - deg) > tolerance && System.currentTimeMillis() < timeout && System.currentTimeMillis() < global_timeout)
          //  turn(deg - getGyroYaw(), 0.8, tolerance * 1.2, (timeout - System.currentTimeMillis()) / Math.pow(10, 3)));
        motorFR.setDirection(FR);
        motorBR.setDirection(BR);
        halt();
        resetPID();
        DbgLog.error(String.format("post-halt: deg:%.2f, gyro:%.2f", deg, getGyroYaw()));
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
            parent_op.waitOneFullHardwareCycle();
        }
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        parent_op.waitOneFullHardwareCycle();
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
            parent_op.waitOneFullHardwareCycle();
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
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        parent_op.waitOneFullHardwareCycle();
    }
    public double getGyroYaw() throws InterruptedException{
        if(IMU == null) {
            DbgLog.error(" IMU is null");
            return -1;
        }
        IMU.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return yawAngle[0];
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
        double kP = 0.15;
        double kI = 0.01;
        double kD = 0.009;
        double PID_change;
        double right;
        double left;
        //double max;
        double target = deg + getGyroYaw();
        while(Math.abs(getGyroYaw() - target) > tolerance && getRuntime() < safety_time && System.currentTimeMillis() < global_timeout)
        {
            PID_change = get_PID(kP, kI, kD);
            //keep the absolute value of the motors above 0.3 and less than 0.7
            right = (Math.signum(PID_change) * Range.clip(Math.abs(PID_change), 0.3, 0.7) / speed_divisor);
            left = -right;
            setRightPower(right);
            setLeftPower(left);
            DbgLog.error(String.format("k*error: %.2f, k*dError: %.2f, k*iError: %.2f", kP * error, kD * dError, kI * iError));
            DbgLog.error(String.format("error: %.2f, dError: %.2f, iError: %.2f", error, dError, iError));
            DbgLog.error(String.format("gyro:%.2f, target:%.2f, PID (right):%.2f", getGyroYaw(), target, PID_change));
        }
        halt();
        halt();
        halt();
        double gyro = getGyroYaw();
        DbgLog.error(String.format("gyro: %.2f, target: %.2f, tolerance: %.2f", gyro, target, tolerance));
        if((Math.abs(gyro - target) > tolerance) && System.currentTimeMillis() < safety_time)
            PID_turn(target - gyro, tolerance, safety_time - getRuntime(), speed_divisor * 1.11);
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
            parent_op.waitOneFullHardwareCycle();
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

            parent_op.waitOneFullHardwareCycle();
            /*if(enableCamera && beacon.getAnalysisMethod().equals(Beacon.AnalysisMethod.COMPLEX))
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
    public void PID_move_camera(double encoder, double target_heading, double speed, boolean enableCamera, long timeout) throws InterruptedException {
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
        double gyro_yaw = getGyroYaw();
        double new_yaw = gyro_yaw;
        resetStartTime();
        double safety_time = System.currentTimeMillis() + timeout;
        double start_time = System.currentTimeMillis();
        double PID_change = 0;
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
            DbgLog.error(String.format("i start: %.2f", iError));
            new_yaw = getGyroYaw();
            if(gyro_yaw != new_yaw)  // leave gyro and PID_change constant if gyro didn't change
            {
                gyro_yaw = new_yaw;
                if (!enableCamera)
                    PID_change = get_PID();
                else {
                    kP = this.kP * 1.2;
                    kI = this.kI;
                    kD = this.kD;
                    PID_change = get_PID(kP, kI, kD) * 1.2 * speed; // scales it to be proportional to the speed
                }
            }
            right = speed + PID_change;
            left = speed / 1.5 - PID_change;
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
            //DEBUG LOGGING
            DbgLog.error(String.format("i end: %.2f", iError));
            tele.addData("a: ", String.format("k*error: %.2f, k*dError: %.2f, k*iError: %.2f", kP * error, kD * dError, kI * iError));
            tele.addData("b: ", String.format("left: %.2f, right: %.2f", left, right));
            tele.addData(" ", String.format("gyro yaw: %.2f, target: %.2f", gyro_yaw, this.target_heading));
            DbgLog.error(String.format("k*error: %.2f, k*dError: %.2f, k*iError: %.2f, PID: %.2f", kP * error, kD * dError, kI * iError, PID_change));
            DbgLog.error(String.format("left: %.2f, right: %.2f, gyro: %.2f, target: %.2f", left, right, gyro_yaw, this.target_heading));
            currentEncoder = mid2(motorBL.getCurrentPosition() - currentBLPosition, motorFL.getCurrentPosition() - currentFLPosition, motorBR.getCurrentPosition() - currentBRPosition, motorBR.getCurrentPosition() - currentBRPosition /*motorBR.getCurrentPosition() - 0 currentBRPosition*/);
            DbgLog.error("currentEncoder: " + currentEncoder + ", target_enc: " + String.format("%.2f",encoder));
            DbgLog.error("FR: "+ String.format("%d, FL: %d", motorFR.getCurrentPosition() - currentFRPosition, motorFL.getCurrentPosition() - currentFLPosition));
            DbgLog.error("BR: " + String.format("%d, BL: %d", motorBR.getCurrentPosition() - currentBRPosition, motorBL.getCurrentPosition() - currentBLPosition));
            //END DEBUG LOGGING

            parent_op.waitOneFullHardwareCycle();
            /*if(enableCamera && beacon.getAnalysisMethod().equals(Beacon.AnalysisMethod.COMPLEX))
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
    public double get_PID() throws InterruptedException
    {
        return get_PID(getGyroYaw(), kP, kI, kD);
    }
    public double get_PID(double kP, double kI, double kD) throws InterruptedException
    {
        return get_PID(getGyroYaw(), kP, kI, kD);
    }
    public double get_PID(double gyro, double kP, double kI, double kD) throws InterruptedException
    {
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
        return kP * error + kD * dError + kI * iError;
    }
    public void resetPID() throws InterruptedException
    {
        if(Math.abs(target_heading - getGyroYaw() + 360) < Math.abs(target_heading - getGyroYaw()))
            error = target_heading - getGyroYaw() + 360;
        else
            error = target_heading - getGyroYaw();
        time = getRuntime();
        iError = 0;
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
        return NewRobotics.calculate_heading(new DPoint(beacon.getAnalysis().getCenter().x, beacon.getAnalysis().getCenter().y), millis / Math.pow(10, 3));
        //return NewRobotics.calculate_heading(DPoint.makeDPoint(beacon.getAnalysis().getCenter()), t / Math.pow(10, 6));
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
    public double find_Beacon_Heading() throws InterruptedException
    {
        resetStartTime();
        double total_heading = 0;
        double cnt = 0;
        double gyro_yaw;
        while(getRuntime() < 5)
        {
            if(beacon.getAnalysisMethod().equals(Beacon.AnalysisMethod.COMPLEX))
            {
                gyro_yaw = getGyroYaw();
                DbgLog.error("Beacon Color" + beacon.getAnalysis().getColorString());
                DbgLog.error("Beacon Location (Center)" + beacon.getAnalysis().getLocationString());
                DbgLog.error("Beacon Confidence" + beacon.getAnalysis().getConfidenceString());
                //DbgLog.error("New Heading: " + NewRobotics.calculate_heading(new DPoint(beacon.getAnalysis().getCenter().y, beacon.getAnalysis().getCenter().x)) + String.format(" old: %.2f", getGyroYaw()));
                DbgLog.error("Relative " + String.format("x: %.2f y: %.2f", beacon.getAnalysis().getCenter().y / 480.0, beacon.getAnalysis().getCenter().x / 864.0));
                //DbgLog.error("New Heading" + NewRobotics.calculate_heading(new DPoint(beacon.getAnalysis().getCenter().y, beacon.getAnalysis().getCenter().x)));
                DbgLog.error(String.format("new heading target xy: %.2f, new yx: %.2f, old heading: %.2f", gyro_yaw +  getHeading(), gyro_yaw + NewRobotics.calculate_heading(new DPoint(beacon.getAnalysis().getCenter().x, beacon.getAnalysis().getCenter().y), 0), gyro_yaw));
                if(beacon.getAnalysis().getConfidence() > 0.2) { // beacon.getAnalysis().getConfidence() > 0.1 && <--- use this if COMPLEX analysis is on
                    total_heading += gyro_yaw + getHeading(0);
                    cnt++;
                }
            }
            parent_op.waitOneFullHardwareCycle();
        }
        if(cnt > 0)
            return (total_heading / cnt);
        else
            return getGyroYaw();
    }
    double makePositive(double heading_Deg)
    {
        return heading_Deg < 0? heading_Deg + 360: heading_Deg;
    }
    void setRightPower(double power)
    {
        motorBR.setPower(Range.clip(power, -1, 1));
        motorFR.setPower(Range.clip(power, -1, 1));
    }
    void setLeftPower(double power)
    {
        motorBL.setPower(Range.clip(-power, -1, 1));
        motorFL.setPower(Range.clip(-power, -1, 1));
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
        while(getRuntime() < 1) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        //encoderTurn(45, 1);
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);
        double heading = getGyroYaw();
        PID_move(squares_to_Encoder(8 * Math.sqrt(2)), heading, 1, false);
        resetStartTime();
        while(getRuntime() < 1) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        /*PID_move(squares_to_Encoder(1 * (Math.sqrt(2))), heading, 1, true);
        resetStartTime();
        while(getRuntime() < 2) // wait 2 seconds*/
        turn(-12, 0.2);
        tele.addData(" ", String.format("gyro yaw: %.2f", getGyroYaw()));
        resetStartTime();
        while(getRuntime() < 1) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        //encoderTurn(-45, 1);
        DbgLog.error(String.format("%.2f", getGyroYaw()));
        PID_move(squares_to_Encoder(2.5), getGyroYaw(), 1, true, 4000);// , 0.5);
        resetStartTime();
        halt();
        dump_Climbers();
        halt();
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
        while(getRuntime() < 1) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        //encoderTurn(45, 1);
        double heading = getGyroYaw();
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);
        PID_move_displacement_cartesian(1500, -1800, speed, 10);
        resetStartTime();
        while(getRuntime() < 1) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        //turn(-20, speed - 0.2);
        PID_turn(-20, 2);
        tele.addData(" ", String.format("gyro yaw: %.2f", getGyroYaw()));
        tele.addData("Before the move into the beacon", " ");
        resetStartTime();
        while(getRuntime() < 8) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        double curr_heading = getGyroYaw();
        DbgLog.error(String.format("curr: %.2f", curr_heading));
        PID_move(squares_to_Encoder(2.5), curr_heading, speed, true, 4000);// , 0.5);
        resetStartTime();
        halt();
        dump_Climbers();
        halt(); halt(); halt();
        resetStartTime();
        while (getRuntime() < 5) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        DbgLog.error(String.format("curr: %.2f", curr_heading));
        PID_move(-squares_to_Encoder(0.3), curr_heading, 0.3, false, 1000);
        resetStartTime();
        while(getRuntime() < 5) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        //turn(curr_heading - getGyroYaw(), speed - 0.2);
        DbgLog.error(String.format("curr - gyro:%.2f", curr_heading - getGyroYaw()));
        PID_turn(curr_heading - getGyroYaw(), 3);
        resetStartTime();
        while(getRuntime() < 5) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        if(beacon.getAnalysis().isLeftKnown() && beacon.getAnalysis().isLeftBlue())
        {
            push_Right();
        }
        else if(beacon.getAnalysis().isRightKnown() && beacon.getAnalysis().isRightBlue())
        {
            push_Left();
        }
    }
    public void Close_Red_Buttons_Polar()
    {

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
        while(getRuntime() < 1) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        //encoderTurn(45, 1);
        double heading = getGyroYaw();
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);
        PID_move_displacement_polar(squares_to_Encoder(long_encoderB + 1 * Math.sqrt(2)), heading, speed);
        resetStartTime();
        while(getRuntime() < 1) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        turn(-20, speed - 0.2);
        tele.addData(" ", String.format("gyro yaw: %.2f", getGyroYaw()));
        tele.addData("Before the move into the beacon" , " ");
        resetStartTime();
        while(getRuntime() < 8) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        DbgLog.error(String.format("%.2f", getGyroYaw()));
        PID_move(squares_to_Encoder(2.5), getGyroYaw(), speed, true, 4000);// , 0.5);
        resetStartTime();
        halt();
        dump_Climbers();
        halt(); halt(); halt();
        if(beacon.getAnalysis().isLeftKnown() && beacon.getAnalysis().isLeftBlue())
        {
            push_Right();
        }
        else if(beacon.getAnalysis().isRightKnown() && beacon.getAnalysis().isRightBlue())
        {
            push_Left();
        }
    }
    public void reverse() throws InterruptedException
    {
        PID_move(-squares_to_Encoder(0.3), getGyroYaw(), 0.5, false);
        turn(-170, 0.8);
        PID_move(-squares_to_Encoder(0.3), getGyroYaw(), 0.5, false);
    }
    public void push_Left() throws InterruptedException
    {
        reverse();
        resetStartTime();
        servoButtonL.setPosition(1);
        while (getRuntime() < 5)
        {
            parent_op.waitOneFullHardwareCycle();
        }
        servoButtonL.setPosition(0.5);
    }
    public void push_Right() throws InterruptedException
    {
        reverse();
        resetStartTime();
        servoButtonR.setPosition(1);
        while (getRuntime() < 5)
        {
            parent_op.waitOneFullHardwareCycle();
        }
        servoButtonR.setPosition(0.5);
    }

    public void Close_Blue_Buttons_CV_NoDump() throws InterruptedException {
        PID_move(squares_to_Encoder(0.75), 0, 1, false);// , 1);
        tele.addData("we made it: ", "yay");
        //halt();
        //ssleep(1000);

        turn(-32.5, 0.75);//turn(-23, 1);
        resetStartTime();
        while(getRuntime() < 1) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        //encoderTurn(45, 1);
        double heading = getGyroYaw();
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);
        PID_move(squares_to_Encoder(long_encoderB + 1 * Math.sqrt(2)), heading, 1, false);
        resetStartTime();
        while(getRuntime() < 1) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        /*PID_move(squares_to_Encoder(1 * (Math.sqrt(2))), heading, 1, true);
        resetStartTime();
        while(getRuntime() < 2) // wait 2 seconds*/
        turn(-59, 0.75);
        tele.addData(" ", String.format("gyro yaw: %.2f", getGyroYaw()));
        resetStartTime();
        while(getRuntime() < 1) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
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
        while(getRuntime() < 1) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        //encoderTurn(45, 1);
        double heading = getGyroYaw();
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);
        PID_move(squares_to_Encoder(long_encoderR + 1 * Math.sqrt(2)), heading, 1, false);
        resetStartTime();
        while(getRuntime() < 1) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        /*PID_move(squares_to_Encoder(1 * (Math.sqrt(2))), heading, 1, true);
        resetStartTime();
        while(getRuntime() < 2) // wait 2 seconds*/
        turn(46, 0.75);
        tele.addData(" ", String.format("gyro yaw: %.2f", getGyroYaw()));
        resetStartTime();
        while(getRuntime() < 1) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
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
        while(getRuntime() < 1) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        //encoderTurn(45, 1);
        double heading = getGyroYaw();
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);
        PID_move(squares_to_Encoder(long_encoderR + 1 * Math.sqrt(2)), heading, 1, false);
        resetStartTime();
        while(getRuntime() < 1) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        /*PID_move(squares_to_Encoder(1 * (Math.sqrt(2))), heading, 1, true);
        resetStartTime();
        while(getRuntime() < 2) // wait 2 seconds*/
        turn(59, 0.75);
        tele.addData(" ", String.format("gyro yaw: %.2f", getGyroYaw()));
        resetStartTime();
        while(getRuntime() < 1) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
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
        parent_op.waitOneFullHardwareCycle();
        servoClimberArm.setPosition(0.06);
        resetStartTime();
        while(getRuntime() < 2) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        servoClimberArm.setPosition(0.17);
        resetStartTime();
        while(getRuntime() < 1) // wait 1 seconds
            parent_op.waitOneFullHardwareCycle();
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
        /*while(getRuntime() < 15)
            waitOneFullHardwareCycle();*/
        halt();
        PID_move(squares_to_Encoder(1.75 * Math.sqrt(2)), 0, 1, false);// , 1);
        resetStartTime();
        while(getRuntime() < 1)
            parent_op.waitOneFullHardwareCycle();
        PID_move(squares_to_Encoder(-0.75 * Math.sqrt(2)), 0, 1, false);
        resetStartTime();
        while(getRuntime() < 1)
            parent_op.waitOneFullHardwareCycle();
        resetStartTime();
        turn(-70, 0.75);
        while(getRuntime() < 1)
            parent_op.waitOneFullHardwareCycle();
        resetStartTime();
        while(getRuntime() < 2) {
            parent_op.waitOneFullHardwareCycle();
            setLeftPower(-1);
            setRightPower(-1);
        }
        halt();

    }
    public void Regionals_Red_Side_Ramp() throws InterruptedException
    {
        halt();
        resetStartTime();
        /*while(getRuntime() < 17)
            waitOneFullHardwareCycle();*/
        PID_move(squares_to_Encoder(0.75), 0, 1, false);// , 1);
        tele.addData("we made it: ", "yay");
        //halt();
        //ssleep(1000);

        turn(32.5, 0.75);//turn(-23, 1);
        resetStartTime();
        while(getRuntime() < 0.5) // wait 0.5 seconds
            parent_op.waitOneFullHardwareCycle();
        //encoderTurn(45, 1);
        double heading = getGyroYaw();
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);
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
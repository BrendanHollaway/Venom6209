package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Venom6209 on 10/5/2015.
 */
public class AutonomousSegments extends LinearOpMode2 {

    UltrasonicSensor ultra;
    UltrasonicSensor frontUltra;
    protected double kP=0.008;
    protected double kI=0.008;
    protected double kD=0.01;


    double cm_rotation = 4*Math.PI*2.54;
    double square_per_rot = 60.0/cm_rotation;                  //different units used for measuring distance moved
    double inches = 1.5*Math.PI;
    double degrees = 2000.0/90.0;
    double xPos = 0;
    double yPos = 0;


    public AutonomousSegments()
    {
        this.motorFL = null;
        this.motorBL = null;                        //"Why not!?" -Brendan
        this.motorFR = null;
        this.motorBR = null;
    }

    public AutonomousSegments(DcMotor motorBL, DcMotor motorBR, AdafruitIMU IMU)
    {
        this.motorBL = motorBL;
        this.motorBR = motorBR;
        //this.IMU = IMU;
    }

    public AutonomousSegments(DcMotor motorFL, DcMotor motorBL, DcMotor motorFR, DcMotor motorBR, AdafruitIMU IMU)
    {
        this.motorFL = motorFL;
        this.motorBL = motorBL;                     //Actually initializes motors
        this.motorFR = motorFR;
        this.motorBR = motorBR;
    }
    /* public void setupMotors () {
        motorFL = hardwareMap.dcMotor.get("motor_1");
        motorBL = hardwareMap.dcMotor.get("motor_2");
        motorFR = hardwareMap.dcMotor.get("motor_3");
        motorBR = hardwareMap.dcMotor.get("motor_4");
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
    } */

    public void ssleep(long ms) throws InterruptedException                 //method for sleeping
    {
        try {
            sleep(ms);
        }
        catch (Exception E){}
    }
    public void updatePosition(double encoderVal) {
        xPos += (motorBR.getCurrentPosition() - encoderVal)*Math.cos(getGyroYaw()); // encoder ticks
        yPos += (motorBR.getCurrentPosition() - encoderVal)*Math.sin(getGyroYaw());
        encoderVal = motorBR.getCurrentPosition();
    }
    //public void moveToPosition()

    public void move(double squares, double speed) throws InterruptedException        //move in a straight line
    {
        double position = squares / square_per_rot * 1120; //1120 is number of encoder ticks per rotation
        int currentPosition = motorBR.getCurrentPosition();                            //measures current encoder value
        while(Math.abs(motorBR.getCurrentPosition()) < position + currentPosition ) {  //moves until encoders change by value inputted
            motorFL.setPower(Math.signum(position) * Math.abs(speed));
            motorBL.setPower(Math.signum(position) * Math.abs(speed));                 //takes sign of position, so sign of speed does not matter
            motorFR.setPower(Math.signum(position) * Math.abs(speed));
            motorBR.setPower(Math.signum(position) * Math.abs(speed));
        }
        halt();
    }
    public void moveCheck(double position, double speed) throws InterruptedException   //move while checking for other robots
    {
        int currentPosition = motorBR.getCurrentPosition();
        while(Math.abs(motorBR.getCurrentPosition()) < position + currentPosition ) {
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
            ssleep(3000);                                 //waits for robot to get out of the way
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
        speed = Range.clip(speed, -1, 1);
        deg %= 360.0;
        if(deg > 180)
        {
            deg -= 360;
        }
        else if(deg < -180)
        {
            deg += 360;
        }
        while(Math.abs(getGyroYaw()) < Math.abs(deg))
        {
            motorFL.setPower(Math.signum(deg) * Math.abs(speed));
            motorBL.setPower(Math.signum(deg) * Math.abs(speed));
            motorFR.setPower(-Math.signum(deg) * Math.abs(speed));
            motorBR.setPower(-Math.signum(deg) * Math.abs(speed));
        }
        halt();
    }
    public void encoderTurn(double position, double speed) throws InterruptedException           //left is negative, right is positive
    {
        speed = Range.clip(speed, -1, 1);
        int currentPosition = motorBR.getCurrentPosition();                                   // WRITE MEDIAN FUNCTION
        if(Math.abs(motorBR.getCurrentPosition()) < position * degrees + currentPosition) {
            while (Math.abs(motorBR.getCurrentPosition()) < position * degrees + currentPosition) {
                //  motorFL.setPower(Math.signum(position) * Math.abs(speed));
                motorBL.setPower(Math.signum(position) * Math.abs(speed));
                motorFR.setPower(-Math.signum(position) * Math.abs(speed));
                motorBR.setPower(-Math.signum(position) * Math.abs(speed));
            }
        }
        else {
            while (Math.abs(motorBR.getCurrentPosition()) > position * degrees + currentPosition) {
                motorFL.setPower(Math.signum(position) * Math.abs(speed));
                motorBL.setPower(Math.signum(position) * Math.abs(speed));
                motorFR.setPower(-Math.signum(position) * Math.abs(speed));
                motorBR.setPower(-Math.signum(position) * Math.abs(speed));
            }
        }
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
    public void halt() {
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }
    public double getGyroYaw() {
        //IMU.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return yawAngle[0];
    }
    public void squareTest() throws InterruptedException {
        move( 1, 1);
    }
    //SOS is deprecated here
    public void SOS(double acc_y, double acc_z)
    {
        //IMU.getAccel(accel);
        acc_y = accel[1];
        acc_z = accel[2];
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
        move(-2 , 1);
        turn(45, 1);
        //encoderTurn(45,1);
        move(-1.5 * (Math.sqrt(2)) , 1);
        turn(45, 1);
        //encoderTurn(45, 1);
        move(-0.5 , 1);
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

    public void move_pos(double x, double y, double speed)
    {
        speed = Range.clip(speed, -1, 1);
        double target_heading;
        double target_x;
        double target_y;
        double left_speed;
        double right_speed;
        double p = 2.0; // the P of PID- position control
        double old_x_acc = 0;
        double curr_x_acc = 0;
        double old_y_acc = 0;
        double curr_y_acc = 0;                                //x is currently forwards and backwards
        double x_vel = 0;                                     //moves robot to position using encoder values, gyro, accelerometer, and magnetometer
        double y_vel = 0;
        double oldEncoder;
        double currEncoder = motorBR.getCurrentPosition();
        double dE;
        double dEx = 0;
        double dEy = 0;
        double oldTime;
        double currTime = getRuntime();
        double dt = 0;
        double[] accs = new double[3];
        while(Math.abs(x-xPos) > .1 || Math.abs(y-yPos) > .1)
        {
            //TARGET HEADING CALCULATIONS
            target_x = x - xPos;
            target_y = y - yPos;
            target_heading = Math.atan2(target_y, target_x);
            target_heading = target_heading > 0? target_heading : target_heading + 2 * Math.PI; // makes it positive, from 0 to 2pi
            target_heading = Math.toDegrees(target_heading); // rad -> deg; 0 to 360
            if(speed < 0) // makes it negative for negative movement... not sure if this will work. Don't back up :D
                target_heading = -target_heading;
            //TIME CALCULATIONS
            oldTime = currTime;
            currTime = getRuntime();
            dt = currTime - oldTime; // puts time in seconds
            //ECNCODER VALUE CALCULATIONS
            oldEncoder = currEncoder;
            currEncoder = motorBR.getCurrentPosition();
            dE = currEncoder - oldEncoder;
            dEx = dE * Math.cos(Math.toRadians(getGyroYaw()));
            dEy = dE * Math.sin(Math.toRadians(getGyroYaw()));
            // ACCELERATION VALUE CALCULATIONS
            //IMU.getAccel(accs);
            old_x_acc = curr_x_acc;
            old_y_acc = curr_y_acc;
            curr_x_acc = accs[0];
            curr_y_acc = accs[1];
            //VELOCITY
            x_vel += 0.5 * (old_x_acc + curr_x_acc) * dt;
            y_vel += 0.5 * (old_y_acc + curr_y_acc) * dt;
            //CALCULATE POSITION
            if(0.5 * (0.5 * (old_x_acc + curr_x_acc)) * Math.pow(dt,2) + x_vel * dt < dEx * 0.5) // 1/2*a*t^2 + v*t < half of change in position encoder-wise;
                xPos += 0.5 * (0.5 * (old_x_acc + curr_x_acc)) * Math.pow(dt,2) + x_vel * dt;    // aka a wheel is free-spinning
            else
                xPos += dEx;
            if(0.5 * (0.5 * (old_y_acc + curr_y_acc)) * Math.pow(dt,2) + y_vel * dt < dEy * 0.5)
                yPos += 0.5 * (0.5 * (old_y_acc + curr_y_acc)) * Math.pow(dt,2) + y_vel * dt;
            else
                yPos += dEy;
            //CALCULATE SPEED
            if(getGyroYaw() < target_heading)
            {
                left_speed = speed;
                right_speed = speed / (1.0 + (Math.abs(target_heading - getGyroYaw()) / p));
            }
            else
            {
                right_speed = speed;
                left_speed = speed / (1.0 + (Math.abs(target_heading - getGyroYaw()) / p));
            }
            if(speed < 0) // if its going backwards, adjustments must be flipped
            {
                double temp = right_speed;
                right_speed = left_speed;
                left_speed = temp;
            }

            motorBL.setPower(left_speed);
            motorBR.setPower(right_speed);
        }
    }
    public void move_To(double squares, double speed, double current_Time_Remaining)
    {
        //double target_heading;
        double target_x;
        double target_y;
        double left_speed;
        double right_speed;
        double p = 2.0; // the P of PID- position control
        double old_x_acc = 0;
        double curr_x_acc = 0;
        double old_y_acc = 0;
        double curr_y_acc = 0;                                //x is currently forwards and backwards
        double x_vel = 0;                                     //moves robot to position using encoder values, gyro, accelerometer, and magnetometer
        double y_vel = 0;
        double oldEncoder;
        double currEncoder = motorBR.getCurrentPosition();
        telemetry.addData("currEncoder", currEncoder);
        double dE;
        double dEx = 0;
        double dEy = 0;
        long oldTime;
        long currTime = System.nanoTime();
        double dt = 0;
        double[] accs = new double[3];
        //double avg_encoder = (motorBL.getCurrentPosition() + motorBR.getCurrentPosition()) / 2.0;
        double encoder = squares / square_per_rot * 1140;
        double target_heading = getGyroYaw();
        while(Math.abs(currEncoder) < Math.abs(encoder) &&  getRuntime() < current_Time_Remaining)
        {
            if(makePositive(getGyroYaw()) < target_heading)
            {
                left_speed = speed;
                right_speed = speed / (1.0 + (Math.abs(target_heading - getGyroYaw()) / p));
            }
            else
            {
                right_speed = speed;
                left_speed = speed / (1.0 + (Math.abs(target_heading - getGyroYaw()) / p));
            }
            motorBR.setPower(Range.clip(-right_speed, -1, 1));
            motorBL.setPower(Range.clip(left_speed, -1, 1));
            telemetry.addData("left_speed", left_speed);
            telemetry.addData("right_speed", left_speed);
        }
    }
    //===========================================================
    //========================PID CONTROL========================
    //===========================================================
    public void PID_move(double encoder, double target_heading, double speed)
    {
        speed = Range.clip(speed, -1, 1);
        motorBR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //All errors are in terms of heading: the robot's yaw
        double prevError;
        double error = 0;
        double iError = 0;
        double dError;
        double max;
        double min;
        double dt;
        resetStartTime();
        double time = getRuntime();
        double PID_change;
        double right;
        double left;
        while(Math.abs(motorBR.getCurrentPosition()) < encoder)
        {
            dt = getRuntime() - time;
            time = getRuntime();
            prevError = error;
            max = Math.max(target_heading, getGyroYaw());
            min = Math.min(target_heading, getGyroYaw());
            error = Math.min(max-min, min + 360 - max);
            if(error == min + 360 - max)
                error *= -1;
            dError = (error - prevError) / dt;
            iError = Range.clip(iError + error * dt,-125,125);
            PID_change = kP * error + kD * dError + kI * iError;
            right = speed - PID_change;
            left = speed + PID_change;
            max = Math.max(right, left);
            right /= max;
            right *= speed;
            left /= max;
            left *= speed;
            setRightPower(right);
            setLeftPower(left);
        }
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


    public void Close_Blue_Buttons() throws InterruptedException {
        move(-2 , 1);
        turn(-45, 1);
        //encoderTurn(45, 1);
        move(-1.5 * (Math.sqrt(2)) , 1);
        turn(-45, 1);
        //encoderTurn(-45, 1);
        move(-0.5 , 1);
    }
    public void Far_Blue_Buttons() throws InterruptedException {
        move(- 1, 1);
        turn(-45, 1);
        //encoderTurn(-45, 1);
        move(-2 * (Math.sqrt(2)) , 1);
        turn(-45, 1);
        //encoderTurn(-45, 1);
        move(-2 , 1);
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
        move(3.5 , 1);
        turn(-45, 1);
        //encoderTurn(-45, 1);
    }

//RAMP SEGMENTS

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

    public void ButtonClimbers() throws InterruptedException {

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
        move( 1, 1);
        turn(90, 1);
        //encoderTurn(90, 1);
        move(1.5 , 1);
        turn(45,1);
        //encoderTurn(45, 1);
    }
    public void BlueButtons_BlueRamp() throws InterruptedException {
        move(4 , 1);
        turn(-45,1);
        //encoderTurn(-45, 1);
    }



    public void IMUFinder() {
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
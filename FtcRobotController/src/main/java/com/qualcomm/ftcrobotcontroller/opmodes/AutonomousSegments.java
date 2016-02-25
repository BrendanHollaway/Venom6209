package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.DPoint;
import com.qualcomm.ftcrobotcontroller.NewRobotics;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.Range;

import org.lasarobotics.vision.ftc.resq.Beacon;

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
        parent_op = par_op;
    }
    public AutonomousSegments(Telemetry telem)
    {
        tele = telem;
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
        tele.addData("turn has ", "begun");
        tele.addData("deg: ", getGyroYaw());
        //halt();
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        double tolerance = 1;
        speed = 1;//Range.clip(Math.abs(speed), -1, 1);
        deg += getGyroYaw();
        deg %= 180;
        //deg now between -180 and 180
        if (getGyroYaw() < deg) {
            while (getGyroYaw() < deg - 3) {
                tele.addData("deg: ", getGyroYaw());
                motorFR.setPower(-speed);
                motorFL.setPower(speed);
                motorBR.setPower(-speed);
                motorBL.setPower(speed);
                parent_op.waitOneFullHardwareCycle();
            }
            while (getGyroYaw() < deg) {
                motorFR.setPower(-speed / 3.0);
                motorFL.setPower(speed / 3.0);
                motorBR.setPower(-speed / 3.0);
                motorBL.setPower(speed / 3.0);
                    parent_op.waitOneFullHardwareCycle();
            }
        } else {
            while (getGyroYaw() > deg + 3) {
                tele.addData("deg: ", getGyroYaw());
                motorFR.setPower(speed);
                motorFL.setPower(-speed);
                motorBR.setPower(speed);
                motorBL.setPower(-speed);
                    parent_op.waitOneFullHardwareCycle();
            }
            while (getGyroYaw() > deg) {
                motorFR.setPower(speed / 3.0);
                motorFL.setPower(-speed / 3.0);
                motorBR.setPower(speed / 3.0);
                motorBL.setPower(-speed / 3.0);
                    parent_op.waitOneFullHardwareCycle();
            }
        }
        //if (Math.abs(getGyroYaw() - deg) > tolerance)
          //  turn(deg - getGyroYaw(), 0.8, tolerance * 1.2);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        halt();
    }
    public void ssleep(long ms) throws InterruptedException                 //method for sleeping
    {
        try {
            sleep(ms);
        }
        catch (Exception E){}
    }
    public void updatePosition(double encoderVal) throws InterruptedException{
        xPos += (motorBR.getCurrentPosition() - encoderVal)*Math.cos(getGyroYaw()); // encoder ticks
        yPos += (motorBR.getCurrentPosition() - encoderVal)*Math.sin(getGyroYaw());
        encoderVal = motorBR.getCurrentPosition();
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
        motorFR.setTargetPosition((int)position);
        motorBL.setTargetPosition((int)position);
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
    public void turn(double deg, double speed, double tolerance) throws InterruptedException
    {
        speed = Range.clip(Math.abs(speed), -1, 1);
        deg += getGyroYaw();
        deg %= 180;
        //deg now between -180 and 180
        if(getGyroYaw() < deg)
        {
            while(getGyroYaw() < deg - 3)
            {
                tele.addData("deg: ", getGyroYaw());
                motorFR.setPower(-speed);
                motorFL.setPower(speed);
                motorBR.setPower(-speed);
                motorBL.setPower(speed);
            }
            while(getGyroYaw() < deg)
            {
                motorFR.setPower(-speed / 3.0);
                motorFL.setPower(speed / 3.0);
                motorBR.setPower(-speed / 3.0);
                motorBL.setPower(speed / 3.0);
            }
        }
        else
        {
            while(getGyroYaw() > deg + 3)
            {
                tele.addData("deg: ", getGyroYaw());
                motorFR.setPower(speed);
                motorFL.setPower(-speed);
                motorBR.setPower(speed);
                motorBL.setPower(-speed);
            }
            while(getGyroYaw() > deg)
            {
                motorFR.setPower(speed / 3.0);
                motorFL.setPower(-speed / 3.0);
                motorBR.setPower(speed / 3.0);
                motorBL.setPower(-speed / 3.0);
            }
        }
        if(Math.abs(getGyroYaw() - deg) > tolerance)
            turn(deg - getGyroYaw(), 0.8, tolerance * 1.2);
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
    public double getGyroYaw() throws InterruptedException{
        parent_op.waitOneFullHardwareCycle();
        if(IMU == null) {
            DbgLog.error(" IMU is null");
            return -1;
        }
        IMU.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return yawAngle[0];
    }
    public void squareTest() throws InterruptedException {
        move(1, 1);
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

    public void move_pos(double x, double y, double speed) throws InterruptedException
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
    public void move_To(double squares, double speed, double current_Time_Remaining) throws InterruptedException
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
    public void PID_move(double encoder, double target_heading, double speed, boolean enableCamera) throws InterruptedException
    {
        PID_move(encoder, target_heading, speed, enableCamera, 12000L);
    }
    public void PID_move(double encoder, double target_heading, double speed, boolean enableCamera, long timeout) throws InterruptedException
    {
        speed = Range.clip(speed, -1, 1);
        this.target_heading = target_heading;
        //motorFL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //motorFL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //All errors are in terms of heading: the robot's yaw
        double max;
        resetStartTime();
        double safety_time = System.currentTimeMillis() + timeout;
        double PID_change;
        double right;
        double left;
        motorFL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int currentFLPosition = motorFL.getCurrentPosition();                            //measures current encoder value
        int currentFRPosition = motorFR.getCurrentPosition();                            //measures current encoder value
        int currentBLPosition = motorBL.getCurrentPosition();                            //measures current encoder value
        int currentBRPosition = motorBR.getCurrentPosition();                            //measures current encoder value*/
        int currentEncoder = 0; //=mid2(currentBLPosition, currentBRPosition, currentFLPosition, currentFRPosition);
        while(Math.abs(currentEncoder) < Math.abs(encoder) && System.currentTimeMillis() < safety_time && System.currentTimeMillis() < timeeout)  //moves until encoders change by value inputted
        {
            PID_change = get_PID();
            right = speed + PID_change;
            left = speed / 3.0 - PID_change;
            max = Math.max(Math.abs(right), Math.abs(left));
            //This standardizes the speeds, so that they are correct relative to each other,
            //and that one of the two will be equivalent, and neither greater, than "speed"
            right /= max;
            right *= speed;
            left /= max;
            left *= speed;
            setRightPower(right);
            setLeftPower(left);

            //DEBUG LOGGING
            tele.addData("a: ", String.format("error: %.2f, dError: %.2f, iError: %.2f", error, dError, iError));
            tele.addData("b: ", String.format("left: %.2f, right: %.2f, gyro: %.2f", left, right, getGyroYaw()));
            tele.addData(" ", String.format("gyro yaw: %.2f, target: ", getGyroYaw(), this.target_heading));
            DbgLog.error(String.format("error: %.2f, dError: %.2f, iError: %.2f", error, dError, iError));
            DbgLog.error(String.format("left: %.2f, right: %.2f, gyro: %.2f, target: %.2f", left, right, getGyroYaw(), this.target_heading));
            currentEncoder = mid2(motorBL.getCurrentPosition() - currentBLPosition, motorFL.getCurrentPosition() - currentFLPosition, motorFR.getCurrentPosition() - currentFRPosition, motorFR.getCurrentPosition() - currentFRPosition /*motorBR.getCurrentPosition() - 0 currentBRPosition*/);
            DbgLog.error("currentEncoder: " + currentEncoder + "target: " + String.format("%.2f",encoder));
            DbgLog.error("FR: "+ String.format("%d,FL: %d", motorFR.getCurrentPosition() - currentFRPosition, motorFL.getCurrentPosition() - currentFLPosition));
            DbgLog.error("BR: " + String.format("%d,BL: %d", motorBR.getCurrentPosition() - currentBRPosition, motorBL.getCurrentPosition() - currentBLPosition));
            //END DEBUG LOGGING

            parent_op.waitOneFullHardwareCycle();
            if(currentEncoder > long_encoderB)
                enableCamera = true;
            if(enableCamera && beacon.getAnalysisMethod().equals(Beacon.AnalysisMethod.COMPLEX))
            {
                if(beacon.getAnalysis().getConfidence() > 0.7) { // beacon.getAnalysis().getConfidence() > 0.1 && <--- use this if COMPLEX analysis is on
                    this.target_heading = getGyroYaw() + getHeading();
                    DbgLog.error("new heading target: " + String.format("%.2f", getGyroYaw() + getHeading()));
                }
            }
            else if(enableCamera && beacon.getAnalysis().isBeaconFound()) { // beacon.getAnalysis().getConfidence() > 0.1 && <--- use this if COMPLEX analysis is on
                this.target_heading = getGyroYaw() + getHeading();
                DbgLog.error("new heading target: " + String.format("%.2f", getGyroYaw() +  getHeading()));
            }
        }
        DbgLog.error("Done with drive forward");
        halt();
    }
    public double get_PID() throws InterruptedException
    {
        //TODO: Make sure that the robot correctly handles transitioning over the boundary
        //TODO: between positive and negative 1 and pos/neg 180
        //possible solution: get initial heading as reference angle, and if it drops
        //or increases then you can add a multiple of 360? needs more thought
        dt = getRuntime() - time;
        time = getRuntime();
        prevError = error;
        if(Math.abs(target_heading - getGyroYaw() + 360) < Math.abs(target_heading - getGyroYaw()))
            error = target_heading - getGyroYaw() + 360;
        else
            error = target_heading - getGyroYaw();
        dError = (error - prevError) / dt;
        iError = Range.clip(iError + error * dt, -125, 125) * 0.99;
        return kP * error + kD * dError + kI * iError;
    }
    public void resetPID()
    {
        time = getRuntime();
        iError = 0;
    }
    public void set_target_heading(double target_heading)
    {
        this.target_heading = target_heading;
    }
    public double getHeading()
    {
        return NewRobotics.calculate_heading(DPoint.makeDPoint(beacon.getAnalysis().getCenter()));
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
        timeeout = System.currentTimeMillis() + (long) (29 * Math.pow(10, 3));
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
        dump_Climbers();
        halt();
    }
    long timeeout = Long.MAX_VALUE;

    public void Close_Blue_Buttons_CV_NoDump() throws InterruptedException {
        timeeout = System.currentTimeMillis() + (long) (29 * Math.pow(10, 3));
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
        timeeout = System.currentTimeMillis() + (long) (29 * Math.pow(10, 3));
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
        timeeout = System.currentTimeMillis() + (long) (29 * Math.pow(10, 3));
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
        servoClimberArm.setPosition(climberDump);
        resetStartTime();
        while(getRuntime() < 2) // wait 2 seconds
            parent_op.waitOneFullHardwareCycle();
        servoClimberArm.setPosition(climberReturn);
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
        timeeout = System.currentTimeMillis() + (long) (29 * Math.pow(10, 3)); //29 seconds
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
        timeeout = System.currentTimeMillis() + (long) (29 * Math.pow(10, 3));
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
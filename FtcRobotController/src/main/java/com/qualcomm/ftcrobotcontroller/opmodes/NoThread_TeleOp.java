package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


public class NoThread_TeleOp extends LinearOpMode2{

    //TODO: implement PID control

    //Variables for PID Control
    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    double[] accel = new double[3];
    //double yToggle = 1.0;
    boolean enableSOS = true;
    boolean PID_enabled = true;
    AutonomousSegments auto = new AutonomousSegments(telemetry, this);
    //boolean SOSactive = false;
    //boolean rat360moved = false;

    //instantiate constants for easy access
    double climberDump = 0.19;
    double climberRetract = 0.06;
    double deadzone = 0.1;
    int togglespeed = 25;

    //Toggle for Zipliners
    boolean rZipRun = false;
    boolean lZipRun = false;
    boolean RZipOut = false;
    boolean LZipOut = false;
    int RZipAdd = 0;
    int LZipAdd = 0;

    //Toggle for Climbers
    boolean climberrun = false;
    int climberadd = 0;


    public NoThread_TeleOp() {

    }

    public double gyroTest() {
        IMU.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return yawAngle[0];
    }
    public double gyroPitch() {
        IMU.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return pitchAngle[0];
    }
    public void sleep(int ms) {
        try {
            wait(ms);
        }
        catch (Exception E) {}

    }

    @Override
    public void runOpMode() throws InterruptedException{
        super.map();
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);

        //Saving buttons as variables for greater efficiency
        //Controller 1 variables
        double LeftY = gamepad1.left_stick_y;
        double RightY = gamepad1.right_stick_y;
        double LT = gamepad1.left_trigger;
        double RT = gamepad1.right_trigger;
        boolean Y = gamepad1.y;
        boolean X = gamepad1.x;
        boolean LB = gamepad1.left_bumper;
        boolean RB = gamepad1.right_bumper;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        //Controller 2 variables
        double LeftY2 = gamepad2.left_stick_y;
        double RightY2 = gamepad2.right_stick_y;
        boolean LB2 = gamepad2.left_bumper;
        boolean RB2 = gamepad2.right_bumper;


    /* BUTTON MAPPING
        CONTROLLER 1
            Left Stick = left wheels
            Right Stick = right wheels
            Y Button = climber dumper toggle
            Left and Right Trigger = release the lift ratchets
            DPad Down = shield down
            DPad Up = shield up

        CONTROLLER 2
            Left Stick = extend/retract lift
            Right Stick = change angle of lift
            Left Bumper = zipliner left toggle
            Right Bumper = zipliner right toggle

     */
        while(!opModeIsActive());
        while(opModeIsActive()) {
            // DRIVE CONTROL - Controller 1

            //Base Driving Controls
            if(enableSOS && gyroPitch() > 50)
            {
                motorFR.setPower(1);
                motorFL.setPower(-1);
                motorBR.setPower(1);
                motorBL.setPower(-1);
            }
            else if (Math.abs(LeftY) > deadzone || Math.abs(RightY) > deadzone)
            {
                motorFR.setPower(Math.abs(RightY) > deadzone ? -RightY : 0);
                motorFL.setPower(Math.abs(LeftY) > deadzone ? LeftY : 0);
                motorBR.setPower(Math.abs(RightY) > deadzone ? -RightY : 0);
                motorBL.setPower(Math.abs(LeftY) > deadzone ? LeftY : 0);
            }
            /*
             * Removing because it seems to conflict with Ratchet
            else if(LT > deadzone || RT > deadzone)
            {
                motorBR.setPower(RT > deadzone? RT : 0);
                motorBL.setPower(LT > deadzone? -LT : 0);
            }
            */
            else if(LB && RB) {
                PID_enabled = true;
                motorFR.setPower(-0.75);
                motorFL.setPower(0.75);
                motorBR.setPower(-0.75);
                motorBL.setPower(0.75);
            }
            else {
                PID_enabled = false;
                auto.resetPID();
                motorFR.setPower(0);
                motorFL.setPower(0);
                motorBR.setPower(0);
                motorBL.setPower(0);

            }

            //Ratchet Controls
            if(LT > deadzone && RT > deadzone) {
                servoRatL.setPosition(0);
                servoRatR.setPosition(Range.clip(servoRatR.getPosition() + 0.01, 0, 1));
            }
            else
                servoRatL.setPosition(.5);

            //Climber dump toggle
            if(Y)
                climberrun = true;
            if(climberrun)
                climberadd+=1;
            if(climberadd >= togglespeed)
            {
                climberrun = false;
                climberadd = 0;
                if(dump)
                    servoClimberArm.setPosition(climberRetract);
                else
                    servoClimberArm.setPosition(climberDump);
            }

            //Shield controls
            if(down)
                motorS.setPower(-1);
            else if(up)
                motorS.setPower(1);
                /* Seems useless?
            else if(X && System.currentTimeMillis() % 50 > 25)
                motorS.setPower(1);*/
            else
                motorS.setPower(0);

            //LIFT CONTROL - Controller 2

            //Pulley controls
            motorPL.setPower(Math.abs(LeftY2) > deadzone ? LeftY2 : 0);
            motorPR.setPower(Math.abs(RightY2) > deadzone ? RightY2 : 0);

            //zipliner toggles
            if (RB2) {
                rZipRun = true;
            }
            if(rZipRun)
            {
                RZipAdd+=1;
            }
            if(RZipAdd >= togglespeed)
            {
                rZipRun = false;
                RZipAdd = 0;
                if(RZipOut){
                    servoR.setPosition(0);
                    RZipOut = false;
                }
                else
                {
                    RZipOut = true;
                    ServoR.setPosition(0.82);
                }
            }
            if (LB2) {
                lZipRun = true;
            }
            if(lZipRun)
            {
                LZipAdd+=1;
            }
            if(LZipAdd >= togglespeed)
            {
                lZipRun = false;
                LZipAdd = 0;
                if(LZipOut){
                    servoL.setPosition(1);
                    LZipOut = false;
                }
                else
                {
                    LZipOut = true;
                    ServoL.setPosition(0.22);
                }
            }

            telemetry.addData("left: ", String.format("%.2f, right: %.2f", servoL.getPosition(), servoR.getPosition()));
            telemetry.addData("ratL: ", String.format("%.2f, climber: %.2f", servoRatL.getPosition(), servoClimberArm.getPosition()));
            telemetry.addData("ratR: ", String.format("%.2f", servoRatR.getPosition()));
            telemetry.addData("encoders: ", String.format("BR: %d + FR: %d + BL: %d + FL: %d", motorBR.getCurrentPosition(), motorFR.getCurrentPosition(), motorBL.getCurrentPosition(), motorFL.getCurrentPosition()));
            //telemetry.addData("gyro yaw; ", gyroTest());
            telemetry.addData("gyro pitch: ", gyroPitch());
            waitOneFullHardwareCycle();
        }
        telemetry.addData("Program complete", "hi");
    }
/*
    void SOScheck() {
        if (gyroPitch() < -55 ) {
            SOSactive = true;
            motorBL.setPower(-1);
            motorBR.setPower(1);
            motorFL.setPower(-1);
            motorFR.setPower(1);
            //If the robot is flipping over, then driver control is
            //motorFL.setPower(0);
            //motorFR.setPower(0);
        }
        else
            SOSactive = false;
    }*/
}




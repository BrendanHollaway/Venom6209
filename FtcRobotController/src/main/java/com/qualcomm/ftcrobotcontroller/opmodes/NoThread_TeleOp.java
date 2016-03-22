package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;


public class NoThread_TeleOp extends LinearOpModeCV2 {

    //TODO: implement PID control


    //Variables for PID Control
    double PID_Offset;
    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    double[] accel = new double[3];
    //double yToggle = 1.0;
    boolean enableSOS = true;
    boolean PID_enabled = true;
    //AutonomousSegments auto = new AutonomousSegments(telemetry, this);
    //boolean SOSactive = false;
    //boolean rat360moved = false;

    //instantiate constants for easy access
    double climberDump = 1;
    double climberRetract = 0;
    double deadzone = 0.1;
    double toggle_delay = 0.25;

    //Toggle for Zipliners
    boolean RZipOut = false;
    boolean LZipOut = false;
    double rZipTimer = getRuntime();
    double lZipTimer = getRuntime();

    //Toggle for Climbers
    boolean dump = false;
    double climberTimer = getRuntime();

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
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);

        //Saving buttons as variables for greater efficiency
        //Controller 1 variables


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
        boolean encoder_enabled = true;
        while(!opModeIsActive());
        while(opModeIsActive()) {
            // DRIVE CONTROL - Controller 1
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
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;
            boolean A = gamepad1.a;
            boolean B = gamepad1.b;
    
            //Controller 2 variables
            double LeftY2 = gamepad2.left_stick_y;
            double RightY2 = gamepad2.right_stick_y;
            boolean LB2 = gamepad2.left_bumper;
            boolean RB2 = gamepad2.right_bumper;
            boolean up2 = gamepad2.dpad_up;
            boolean down2 = gamepad2.dpad_down;

            //Base Driving Controls
            if(enableSOS && gyroPitch() > 50)
            {
                motorFR.setPower(1);
                motorFL.setPower(1);
                motorBR.setPower(1);
                motorBL.setPower(1);
            }
            else if (Math.abs(LeftY) > deadzone || Math.abs(RightY) > deadzone)
            {
                motorFR.setPower(Math.abs(RightY) > deadzone ? -RightY : 0);
                motorFL.setPower(Math.abs(LeftY) > deadzone ? -LeftY : 0);
                motorBR.setPower(Math.abs(RightY) > deadzone ? -RightY : 0);
                motorBL.setPower(Math.abs(LeftY) > deadzone ? -LeftY : 0);
            }
            else if(up) {
                PID_Offset = 0;//auto.get_PID();
                motorFR.setPower(Range.clip(-0.75 - PID_Offset, -1, 0));
                motorFL.setPower(Range.clip(-0.75 + PID_Offset, -1, 0));
                motorBR.setPower(Range.clip(-0.75 - PID_Offset, -1, 0));
                motorBL.setPower(Range.clip(-0.75 + PID_Offset, -1, 0));
            }
            else {
                //auto.resetPID();
                motorFR.setPower(0);
                motorFL.setPower(0);
                motorBR.setPower(0);
                motorBL.setPower(0);

            }

            //Ratchet Controls
            if(LT > deadzone && RT > deadzone) {
                servoRatL.setPosition(0);
                servoRatR.setPosition(.58);
            }
            else
            {
                servoRatL.setPosition(.5);
                servoRatR.setPosition(.44);
            }


            //Climber dump toggle
            if(getRuntime() > climberTimer && Y)
            {
                climberTimer = getRuntime() + toggle_delay;
                if(dump)
                    servoClimberArm.setPosition(climberRetract);
                else
                    servoClimberArm.setPosition(climberDump);
                dump = !dump;
            }


            //Shield controls
            if(gamepad1.back)
                encoder_enabled = false;
            if(LB && (!encoder_enabled || motorS.getCurrentPosition() < 500))
                motorS.setPower(1);
            else if(RB && (!encoder_enabled || motorS.getCurrentPosition() > -20000))
                motorS.setPower(-1);
            else if(down2 && (!encoder_enabled || motorS.getCurrentPosition() < 500))
                motorS.setPower(1);
            else if(up2 && (!encoder_enabled || motorS.getCurrentPosition() > -20000))
                motorS.setPower(-1);
            else
                motorS.setPower(0);
            telemetry.addData("motorS position: ", motorS.getCurrentPosition());
            //LIFT CONTROL - Controller 2

            //Pulley controls
            motorPR.setPower(Math.abs(LeftY2) > deadzone ? LeftY2 : 0);
            motorPL.setPower(Math.abs(RightY2) > deadzone ? RightY2 : 0);

            //zipliner toggles
            if(getRuntime() > rZipTimer && RB2)
            {
                rZipTimer = getRuntime() + toggle_delay;
                if(RZipOut)
                    servoR.setPosition(0.82);
                else
                    servoR.setPosition(0);
                RZipOut = !RZipOut;
            }
            else if(gamepad2.x)
            {
                servoR.setPosition(Range.clip(servoR.getPosition() + 0.01, 0, 1));
            }
            else if(gamepad2.y)
            {
                servoR.setPosition(Range.clip(servoR.getPosition() - 0.01, 0, 1));
            }
            if(getRuntime() > lZipTimer && LB2)
            {
                lZipTimer = getRuntime() + toggle_delay;
                if(LZipOut)
                    servoL.setPosition(0.25);
                else
                    servoL.setPosition(1);
                LZipOut = !LZipOut;
            }
            else if(gamepad2.a)
            {
                servoL.setPosition(Range.clip(servoL.getPosition() + 0.01, 0, 1));
            }
            else if(gamepad2.b)
            {
                servoL.setPosition(Range.clip(servoL.getPosition() - 0.01, 0, 1));
            }
            if(A) {
                servoAllClearL.setPosition(1);
                servoAllClearR.setPosition(1);
            }
            else if(B) {
                servoAllClearL.setPosition(0);
                servoAllClearR.setPosition(0);
            }
            else {
                servoAllClearL.setPosition(.5);
                servoAllClearR.setPosition(.5);
            }
            if(left) {
                servoButtonL.setPosition(0);
                servoButtonR.setPosition(0);
            }
            else if(right) {
                servoButtonL.setPosition(1);
                servoButtonR.setPosition(1);
            }
            else {
                servoButtonL.setPosition(.5);
                servoButtonR.setPosition(.5);
            }
            telemetry.addData("x acc: ",String.format("%.2f, y acc: %.2f, z acc: %.2f", accel[0], accel[1], accel[2]));
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




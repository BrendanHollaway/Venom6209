package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;


public class NoThread_TeleOp extends AutonomousSegments {
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
    double climberDump = 0.085;
    double climberRetract = 0.04;
    double deadzone = 0.1;
    double toggle_delay = 0.25;
    double belt_delay = 0.1;

    //Toggle for Zipliners
    boolean RZipOut = false;
    boolean LZipOut = false;
    boolean blue_side = false;
    double SOS_timer = getRuntime();
    double blue_side_timer = getRuntime();
    double rZipTimer = getRuntime();
    double lZipTimer = getRuntime();
    double belt_timer = getRuntime();

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
        super.force_map();
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
        while(!opModeIsActive());
        while(opModeIsActive()) {
            // DRIVE CONTROL - Controller 1
            double LeftY = -gamepad1.left_stick_y;
            double RightY = -gamepad1.right_stick_y;
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
            boolean left2 = gamepad2.dpad_left;
            boolean right2 = gamepad2.dpad_right;
            double LT2 = gamepad2.left_trigger;
            double RT2 = gamepad2.right_trigger;
            boolean X2 = gamepad2.x;
            boolean Y2 = gamepad2.y;
            boolean A2 = gamepad2.a;
            //Base Driving Controls
            if(A && getRuntime() < SOS_timer)
            {
                enableSOS = !enableSOS;
                SOS_timer = getRuntime() + toggle_delay;
            }
            if(enableSOS && gyroPitch() > 50)
            {
                motorFR.setPower(1);
                motorFL.setPower(1);
                motorBR.setPower(1);
                motorBL.setPower(1);
            }
            else if((Math.abs(LeftY) > deadzone || Math.abs(RightY) > deadzone) && Math.signum(RightY) == Math.signum(LeftY))
            {
                motorFR.setPower(Math.abs(RightY) > deadzone ? -RightY : 0);
                motorFL.setPower(Math.abs(LeftY) > deadzone ? -LeftY : 0);
                motorBR.setPower(Math.abs(RightY) > deadzone ? -RightY : 0);
                motorBL.setPower(Math.abs(LeftY) > deadzone ? -LeftY : 0);
            }
            else if((Math.abs(LeftY) > deadzone || Math.abs(RightY) > deadzone))
            {
                motorFR.setPower(Math.abs(RightY) > deadzone ? RightY : 0);
                motorFL.setPower(Math.abs(LeftY) > deadzone ? LeftY : 0);
                motorBR.setPower(Math.abs(RightY) > deadzone ? RightY : 0);
                motorBL.setPower(Math.abs(LeftY) > deadzone ? LeftY : 0);
                telemetry.addData("Turn", "ing");
            }
            else if(up) {
                PID_Offset = get_PID();
                motorFR.setPower(Range.clip(-0.75 - PID_Offset, -1, 0));
                motorFL.setPower(Range.clip(-0.75 + PID_Offset, -1, 0));
                motorBR.setPower(Range.clip(-0.75 - PID_Offset, -1, 0));
                motorBL.setPower(Range.clip(-0.75 + PID_Offset, -1, 0));
            }
            else {
                resetPID();
                motorFR.setPower(0);
                motorFL.setPower(0);
                motorBR.setPower(0);
                motorBL.setPower(0);

            }
            if(LT > deadzone)
                motorM.setPower(LT);
            else if(RT > deadzone)
                motorM.setPower(-RT);
            else
                motorM.setPower(0);
            //Ratchet Controls
            if(B && X) {
                servoRatL.setPosition(0);
                servoRatR.setPosition(.58);
            }
            else
            {
                servoRatL.setPosition(.5);
                servoRatR.setPosition(.46);
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
            if(LB)
                motorS.setPower(1);
            else if(RB)
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
            if(getRuntime() > lZipTimer && LB2)
            {
                lZipTimer = getRuntime() + toggle_delay;
                if(LZipOut)
                    servoL.setPosition(0.25);
                else
                    servoL.setPosition(1);
                LZipOut = !LZipOut;
            }
            if (X2) {
               servoYB5.setPosition(0.3);
            }
            else if (Y2) {
                servoYB5.setPosition(0);
            }
            if(left) {
                servoButtPush.setPosition(0);
            }
            else if (right) {
                servoButtPush.setPosition(1);
            }

            if (left2 && getRuntime() > belt_timer) {
                belt_timer = getRuntime() + belt_delay;
                servoBasketBelt.setPosition(Range.clip(servoBasketBelt.getPosition() + .015, 0, 1));
            }
            else if (right2 && getRuntime() > belt_timer) {
                belt_timer = getRuntime() + belt_delay;
                servoBasketBelt.setPosition(Range.clip(servoBasketBelt.getPosition() - .015, 0, 1));
            }

            if(up2 && !blue_side) {
                servoBasketAngle.setPosition(0.86);
            }
            else if(down2 && !blue_side){
                servoBasketAngle.setPosition(0.27);
            }
            else if(up2 && blue_side)
            {
                servoBasketAngle.setPosition(.9);
            }
            else if(down2 && blue_side)
            {
                servoBasketAngle.setPosition(0.5);
            }
            telemetry.addData("x acc: ",String.format("%.2f, y acc: %.2f, z acc: %.2f", accel[0], accel[1], accel[2]));
            telemetry.addData("left: ", String.format("%.2f, right: %.2f", servoL.getPosition(), servoR.getPosition()));
            telemetry.addData("ratL: ", String.format("%.2f, climber: %.2f", servoRatL.getPosition(), servoClimberArm.getPosition()));
            telemetry.addData("BasketBelt: ", String.format("%.2f Angle: %.2f", servoBasketBelt.getPosition(), servoBasketAngle.getPosition()));
            telemetry.addData("encoders: ", String.format("BR: %d + FR: %d + BL: %d + FL: %d", motorBR.getCurrentPosition(), motorFR.getCurrentPosition(), motorBL.getCurrentPosition(), motorFL.getCurrentPosition()));
            //telemetry.addData("gyro yaw; ", gyroTest());
            telemetry.addData("gyro pitch: ", gyroPitch());
            waitOneFullHardwareCycle();
        }
    }
}




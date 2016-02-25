package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;


public class NoThread_TeleOp extends LinearOpMode2{

    //TODO: implement PID control


    //Variables for PID Control
    double PID_Offset;
    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    double[] accel = new double[3];
    //double yToggle = 1.0;
    boolean enableSOS = true;
    boolean PID_enabled = true;
    AutonomousSegments auto = new AutonomousSegments(telemetry);
    //boolean SOSactive = false;
    //boolean rat360moved = false;

    //instantiate constants for easy access
    double climberDump = 0.19;
    double climberRetract = 0.06;
    double deadzone = 0.1;
    double toggle_delay = (double) TimeUnit.SECONDS.toNanos(1L) * 0.25;

    //Toggle for Zipliners
    boolean RZipOut = false;
    boolean LZipOut = false;
    double rZipTimer = getRuntime();
    double lZipTimer = getRuntime();

    //Toggle for Climbers
    boolean dump = false;
    double climberTimer = getRuntime();


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
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);

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
        boolean up2 = gamepad2.dpad_up;
        boolean down2 = gamepad2.dpad_up;


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
                motorFL.setPower(1);
                motorBR.setPower(1);
                motorBL.setPower(1);
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
                PID_Offset = auto.get_PID();
                PID_motors_set(0.8, PID_Offset);
            }
            else {
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
            if(down)
                motorS.setPower(-1);
            else if(up)
                motorS.setPower(1);
            else if(down2)
                motorS.setPower(-1);
            else if(up2)
                motorS.setPower(1);
            else
                motorS.setPower(0);

            //LIFT CONTROL - Controller 2

            //Pulley controls
            motorPL.setPower(Math.abs(LeftY2) > deadzone ? LeftY2 : 0);
            motorPR.setPower(Math.abs(RightY2) > deadzone ? RightY2 : 0);

            //zipliner toggles
            if(getRuntime() > rZipTimer && LB2)
            {
                rZipTimer = getRuntime() + toggle_delay;
                if(RZipOut)
                    servoR.setPosition(0);
                else
                    servoR.setPosition(0.82);
                RZipOut = !RZipOut;
            }
            if(getRuntime() > lZipTimer && RB2)
            {
                lZipTimer = getRuntime() + toggle_delay;
                if(LZipOut)
                    servoR.setPosition(1);
                else
                    servoR.setPosition(0.22);
                LZipOut = !LZipOut;
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
    public void PID_motors_set(double speed, double PID_change)
    {
        double right = speed + PID_change;
        double left = speed - PID_change;
        double max = Math.max(Math.abs(right), Math.abs(left));
        //This standardizes the speeds, so that they are correct relative to each other,
        //and that one of the two will be equivalent, and neither greater, than "speed"
        right /= max;
        right *= speed;
        left /= max;
        left *= speed;
        motorBL.setPower(left);
        motorFL.setPower(left);
        motorBR.setPower(right);
        motorFR.setPower(right);
    }
}




package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


public class NoThread_TeleOp extends LinearOpMode2{

    double y1_1;
    double y1_2;
    double y2_1;
    double y2_2;
    boolean yButton1;
    boolean yButton2;
    boolean aButton1;
    boolean aButton2;
    boolean xButton2;
    boolean bButton2;
    boolean lBump1;
    boolean rBump1;
    double lTrig1;
    double rTrig1;
    boolean lBump2;
    boolean rBump2;
    double lTrig2;
    double rTrig2;
    boolean dpadUp1;
    boolean dpadDown1;
    boolean dpadUp2;
    boolean dpadDown2;
    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    double[] accel = new double[3];
    double yToggle = 1.0;
    boolean enableSOS = true;
    boolean SOSactive = false;
    boolean rat360moved = false;
    


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
    public boolean buttonheld(boolean button) {
        int holdTime = 0;
        while (button) {
            holdTime++;
            if (holdTime > 4)
                return true;
        }
            return false;
    }
    @Override
    public void runOpMode() {
        super.map();

    /* BUTTON MAPPING
        CONTROLLER 1
            y1 = left tread
            y2 = right tread
            y_btn = toggle for speed of treads
            a_btn = toggle for SOS mode
            left_trigger = swing-out the left zip-line knocker
            left_bumper = swing-in the left zip-line knocker
            right_trigger = swing-out the right zip-line knocker
            right_bumper = swing-in the right zip-line knocker

        CONTROLLER 2
            y1 = extend/retract lift
            y2 = change angle of lift
            y_btn = hold for top ratchet release
            a_btn = hold for bot ratchet release
            left_bumper = retract climber arm towards robot
            right_bumper = flip climber arm out
     */
        while(!opModeIsActive())

        while(opModeIsActive()) {

            y1_1 = gamepad1.left_stick_y;
            y1_2 = -gamepad1.right_stick_y;
            yButton1 = gamepad1.y;
            yButton2 = gamepad2.y;
            aButton1 = gamepad1.a;
            aButton2 = gamepad2.a;
            xButton2 = gamepad2.x;
            bButton2 = gamepad2.b;
            lBump1 = gamepad1.left_bumper;
            rBump1 = gamepad1.right_bumper;
            lTrig1 = gamepad1.left_trigger;
            rTrig1 = gamepad1.right_trigger;
            lBump2 = gamepad2.left_bumper;
            rBump2 = gamepad2.right_bumper;
            lTrig2 = gamepad2.left_trigger;
            rTrig2 = gamepad2.right_trigger;
            dpadUp1 = gamepad1.dpad_up;
            dpadDown1 = gamepad1.dpad_down;
            dpadUp2 = gamepad2.dpad_up;
            dpadDown2 = gamepad2.dpad_down;
            y2_1 = gamepad2.left_stick_y;
            y2_2 = gamepad2.right_stick_y;
        /* if (yButton1) {
            if (yToggle == 1) {
                y_toggle_count++;
            }
            else if (yToggle == 3) {
                yToggle = 1;
            }
            if(y_toggle_count > 1)
            {
                y_toggle_count = 0;
                yToggle = 3;
            }
        } */

            // DRIVE CONTROL AND CLIMBER RELEASE


            if (lTrig1 > 0.1 ) {
                servoL.setPosition(Range.clip(servoL.getPosition() + 0.02, 0, 1));
            } else if (lBump1) {
                servoL.setPosition(Range.clip(servoL.getPosition() - 0.02, 0, 1));
            }
            if (rTrig1 > 0.1) {
                servoR.setPosition(Range.clip(servoR.getPosition() - 0.02, 0, 1));
            } else if (rBump1) {
                servoR.setPosition(Range.clip(servoR.getPosition() + 0.02, 0, 1));
            }
            if (Math.abs(y1_1) > 0.1 && Math.abs(y1_2) > 0) {
                motorFR.setPower(y1_2);
                motorFL.setPower(y1_1);
                motorBR.setPower(y1_2);
                motorBL.setPower(y1_1);
            } else if (Math.abs(y1_1) > 0.1) {
                motorFR.setPower(0);
                motorFL.setPower(y1_1);
                motorBR.setPower(0);
                motorBL.setPower(y1_1);
            } else if (Math.abs(y1_2) > 0.1) {
                motorFR.setPower(y1_2);
                motorFL.setPower(0);
                motorBR.setPower(y1_2);
                motorBL.setPower(0);
            } else {
                motorFR.setPower(0);
                motorFL.setPower(0);
                motorBR.setPower(0);
                motorBL.setPower(0);
            }

            if(Math.abs(y2_1) > 0)
            {
                motorPL.setPower(y2_1);
                motorPR.setPower(-y2_1);
            }
            else if(gamepad2.right_bumper)
                motorPR.setPower(-0.25);
            else if(gamepad2.right_trigger > 0.1)
                motorPR.setPower(0.25);
            else if(gamepad2.left_bumper)
                motorPL.setPower(-0.25);
            else if(gamepad2.left_trigger > 0.1)
                motorPL.setPower(0.25);
            else
            {
                motorPL.setPower(0);
                motorPR.setPower(0);
            }

            if (buttonheld(xButton2))
                servoLRat.setPosition(0);
            if (buttonheld(bButton2) && !rat360moved) {
                servoRRat.setPosition(1);
                sleep(100);
                servoRRat.setPosition(.5);
                rat360moved = true;
            }

            // LIFT CONTROLS START HERE
/*
            if (lBump2 )
                servoClimberArm.setPosition(0);
            else if (rBump2 )
                servoClimberArm.setPosition(1);
*/
            SOScheck();
            telemetry.addData("gyro yaw; ", gyroTest());
            telemetry.addData("gyro pitch: ", gyroPitch());

        }
    }
    /*double scaleInput(double val)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };




        // get the corresponding index for the  array.
        int index = (int) (val * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }


        if (val < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        //if ()

        return dScale;
    } */

    void SOScheck() {
        IMU.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        if (pitchAngle[0] < -55 ) {
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
    }
}




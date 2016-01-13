package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


public class NoThread_TeleOp extends LinearOpMode2{

    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    double[] accel = new double[3];
    //double yToggle = 1.0;
    boolean enableSOS = true;
    //boolean SOSactive = false;
    //boolean rat360moved = false;
    


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
        while(!opModeIsActive());

        while(opModeIsActive()) {

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

            // DRIVE CONTROL - Controller 1
            if(enableSOS && gyroPitch() > 50)
            {
                motorFR.setPower(1);
                motorFL.setPower(-1);
                motorBR.setPower(1);
                motorBL.setPower(-1);
            }
            else if (Math.abs(gamepad1.left_stick_y) > 0.1 && Math.abs(gamepad1.right_stick_y) > 0.1) {
                motorFR.setPower(gamepad1.left_stick_y);
                motorFL.setPower(-gamepad1.right_stick_y);
                motorBR.setPower(gamepad1.left_stick_y);
                motorBL.setPower(-gamepad1.right_stick_y);
            } else if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                motorFR.setPower(gamepad1.left_stick_y);
                motorFL.setPower(0);
                motorBR.setPower(gamepad1.left_stick_y);
                motorBL.setPower(0);
            } else if (Math.abs(gamepad1.right_stick_y) > 0.1) {
                motorFR.setPower(0);
                motorFL.setPower(-gamepad1.right_stick_y);
                motorBR.setPower(0);
                motorBL.setPower(-gamepad1.right_stick_y);
            } else if(gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1)
            {
                motorBR.setPower(gamepad1.right_trigger > 0.1? gamepad1.right_trigger : 0);
                motorBL.setPower(gamepad1.left_trigger > 0.1? -gamepad1.left_trigger : 0);
            }
            else {
                motorFR.setPower(0);
                motorFL.setPower(0);
                motorBR.setPower(0);
                motorBL.setPower(0);
            }
            if(gamepad1.left_bumper && gamepad1.right_bumper)
            {
                servoLRat.setPosition(0);
                servoRRat.setPosition(Range.clip(servoRRat.getPosition() + 0.01, 0, 1));
            }
            else
                servoLRat.setPosition(.5);
            if(Math.abs(gamepad2.left_stick_y) > 0.1 && Math.abs(gamepad2.right_stick_y) > 0.1)
            {
                motorPL.setPower(gamepad2.right_stick_y);
                motorPR.setPower(gamepad2.left_stick_y);
            }
            else if(Math.abs(gamepad2.right_stick_y) > 0.1)
            {
                motorPL.setPower(gamepad2.right_stick_y);
                motorPR.setPower(0);
            }
            else if(Math.abs(gamepad2.left_stick_y) > 0.1)
            {
                motorPL.setPower(0);
                motorPR.setPower(gamepad2.left_stick_y);
            }
            /*else if(gamepad2.right_bumper)
                motorPR.setPower(-0.25);
            else if(gamepad2.right_trigger > 0.1)
                motorPR.setPower(0.25);
            else if(gamepad2.left_bumper)
                motorPL.setPower(-0.25);
            else if(gamepad2.left_trigger > 0.1)
                motorPL.setPower(0.25);*/
            else
            {
                motorPL.setPower(0);
                motorPR.setPower(0);
            }
            // LIFT AND SERVO CONTROL - CONTROLLER 2
            if (gamepad1.dpad_up)
                servoClimberArm.setPosition(1);
            else if (gamepad1.dpad_down)
                servoClimberArm.setPosition(0);
            /*if (xButton2) {
                servoLRat.setPosition(1);
                servoRRat.setPosition(Range.clip(servoRRat.getPosition() - 0.01, 0, 1));
            }
            else*/
            if(gamepad2.x)
                servoF.setPosition(0);
            else if(gamepad2.y)
            {
                servoF.setPosition(1);
            }
            else
                servoF.setPosition(0.5);
            if (gamepad1.x) {
                servoL.setPosition(Range.clip(servoL.getPosition() + 0.015, 0, .98));
            } else if (gamepad1.y) {
                servoL.setPosition(Range.clip(servoL.getPosition() - 0.015, 0, .98));
            }
            if (gamepad1.a) {
                servoR.setPosition(Range.clip(servoR.getPosition() - 0.015, 0, 1));
            } else if (gamepad1.b) {
                servoR.setPosition(Range.clip(servoR.getPosition() + 0.015, 0, 1));
            }
                //sleep(100);
                //servoRRat.setPosition(.5);


            // LIFT CONTROLS START HERE
/*
            if (lBump2 )
                servoClimberArm.setPosition(0);
            else if (rBump2 )
                servoClimberArm.setPosition(1);
*/
            //SOScheck();
            telemetry.addData("left: ", String.format("%.2f, r: %.2f", servoL.getPosition(), servoR.getPosition()));
            telemetry.addData("ratL: ", String.format("%.2f, climb: %.2f", servoLRat.getPosition(), servoClimberArm.getPosition()));
            telemetry.addData("servo: ", String.format("rrat: %.2f", servoRRat.getPosition()));
            telemetry.addData("encoderBL: ", String.format("%d + %d + %d + %d", motorBR.getCurrentPosition(), motorFR.getCurrentPosition(), motorBL.getCurrentPosition(), motorFL.getCurrentPosition()));
            //telemetry.addData("gyro yaw; ", gyroTest());
            telemetry.addData("gyro pitch: ", gyroPitch());
        }
        telemetry.addData("Program complete", " hi");
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




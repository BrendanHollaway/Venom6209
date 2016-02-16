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
    public void runOpMode() throws InterruptedException{
        super.map();
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
    /* BUTTON MAPPING
        CONTROLLER 1
            y1 = left wheels
            y2 = right wheels
            a_btn = retract climber dumper
            y_btn = engage climber dumper
            hold left trigger = spit out blocks
            hold right trigger = suck up blocks
            start = stop manipulator
            hold left and right bumper = release the lift ratchets

        CONTROLLER 2
            y1 = extend/retract lift
            y2 = change angle of lift
            y_btn = zipliner left out
            x_btn = zipliner left in
            b_btn = zipliner right out
            a_btn = zipliner right in
            left_bumper = keep blocks in
            right_bumper = dump blocks
            right_trigger = turn tread right
            left_trigger = turn tread left
            back = stop tread
            dpad_down = shield down
            dpad_up = shield up

     */
        while(!opModeIsActive());

        while(opModeIsActive()) {
            // DRIVE CONTROL - Controller 1
            if(enableSOS && gyroPitch() > 50)
            {
                motorFR.setPower(1);
                motorFL.setPower(-1);
                motorBR.setPower(1);
                motorBL.setPower(-1);
            }
            else if (Math.abs(gamepad1.left_stick_y) > 0.1 && Math.abs(gamepad1.right_stick_y) > 0.1) {
                motorFR.setPower(-gamepad1.right_stick_y);
                motorFL.setPower(gamepad1.left_stick_y);
                motorBR.setPower(-gamepad1.right_stick_y);
                motorBL.setPower(gamepad1.left_stick_y);
            } else if (Math.abs(gamepad1.right_stick_y) > 0.1) {
                motorFR.setPower(-gamepad1.right_stick_y);
                motorFL.setPower(0);
                motorBR.setPower(-gamepad1.right_stick_y);
                motorBL.setPower(0);
            } else if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                motorFR.setPower(0);
                motorFL.setPower(gamepad1.left_stick_y);
                motorBR.setPower(0);
                motorBL.setPower(gamepad1.left_stick_y);
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
            if(gamepad1.left_trigger > .1 && gamepad1.right_trigger > .1) {
                servoRatL.setPosition(0);
                servoRatR.setPosition(Range.clip(servoRatR.getPosition() + 0.01, 0, 1));
            }
            else
                servoRatL.setPosition(.5);
            if (gamepad1.y) {
                servoClimberArm.setPosition(.19);
            }
            else if (gamepad1.a) {
                servoClimberArm.setPosition(.06);
            }
            if(gamepad1.dpad_down)
                motorS.setPower(-1);
            else if(gamepad1.dpad_up)
                motorS.setPower(1);
            else if(gamepad1.x && System.currentTimeMillis() % 50 > 25)
                motorS.setPower(1);
            else
                motorS.setPower(0);
            if (Math.abs(gamepad2.left_stick_y) > 0.1 && Math.abs(gamepad2.right_stick_y) > 0.1)
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
            else
            {
                motorPL.setPower(0);
                motorPR.setPower(0);
            }
            if (gamepad2.right_bumper) {
                servoR.setPosition(.85);
            } else if (gamepad2.right_trigger > .1) {
                servoR.setPosition(0);
            }
            if (gamepad2.left_bumper) {
                servoL.setPosition(0);
            } else if (gamepad2.left_trigger > .1) {
                servoL.setPosition(.94);
            }

            telemetry.addData("left: ", String.format("%.2f, r: %.2f", servoL.getPosition(), servoR.getPosition()));
            telemetry.addData("ratL: ", String.format("%.2f, climb: %.2f", servoRatL.getPosition(), servoClimberArm.getPosition()));
            telemetry.addData("servo: ", String.format("rrat: %.2f", servoRatR.getPosition()));
            telemetry.addData("encoderBL: ", String.format("%d + %d + %d + %d", motorBR.getCurrentPosition(), motorFR.getCurrentPosition(), motorBL.getCurrentPosition(), motorFL.getCurrentPosition()));
            //telemetry.addData("gyro yaw; ", gyroTest());
            telemetry.addData("gyro pitch: ", gyroPitch());
            waitOneFullHardwareCycle();
        }
        telemetry.addData("Program complete", "hi");
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




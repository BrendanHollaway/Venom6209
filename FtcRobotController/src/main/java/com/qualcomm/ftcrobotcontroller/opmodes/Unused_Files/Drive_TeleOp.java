package com.qualcomm.ftcrobotcontroller.opmodes.Unused_Files;

import com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs.BufferSupportedRunnable;
import com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs.BufferedInterpolatedMotor;
import com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs.BufferedMotor;
import com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs.BufferedServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by viperbots on 9/21/2015.
 */
public class Drive_TeleOp extends BufferSupportedRunnable {


        BufferedInterpolatedMotor frontRight;
        BufferedInterpolatedMotor frontLeft;
        BufferedInterpolatedMotor backRight;
        BufferedInterpolatedMotor backLeft;
        BufferedServo bucket;
        Gamepad localGamepad1;
        Gamepad localGamepad2;
        Object gamepadLock = new Object();
        double yToggle = 1.0;
        double dScale = 0.0;

        public Drive_TeleOp(HardwareMap map) {
            super(map);

            frontRight = BufferedInterpolatedMotor.createBufferedInterpolatedMotor(map, "Left");
            frontRight.attachEncoder();
            frontRight.hardwareResetBothAttachedEncodersValues();

            frontLeft = BufferedInterpolatedMotor.createBufferedInterpolatedMotor(map, "Right");
            frontLeft.attachEncoder();
            frontLeft.hardwareResetBothAttachedEncodersValues();

            frontRight.setBufferMode(BufferedMotor.BUFFER_METHOD.POWER_AND_ENCODER);
            frontRight.resetEncoderValue();
            frontLeft.setBufferMode(BufferedMotor.BUFFER_METHOD.POWER_AND_ENCODER);
            frontLeft.resetEncoderValue();

            backRight = BufferedInterpolatedMotor.createBufferedInterpolatedMotor(map, "Left");
            backRight.attachEncoder();
            backRight.hardwareResetBothAttachedEncodersValues();

            backLeft = BufferedInterpolatedMotor.createBufferedInterpolatedMotor(map, "Right");
            backLeft.attachEncoder();
            backLeft.hardwareResetBothAttachedEncodersValues();

            backRight.setBufferMode(BufferedMotor.BUFFER_METHOD.POWER_AND_ENCODER);
            backRight.resetEncoderValue();
            backLeft.setBufferMode(BufferedMotor.BUFFER_METHOD.POWER_AND_ENCODER);
            backLeft.resetEncoderValue();

            bucket = BufferedServo.createBufferedMotor(map, "bucket");
        }

        @Override
        public void newDataReceived() { // NOTE: Not called from this thread.  Called from opMode thread
            super.newDataReceived();
        }

        public void passNewJoystickInfo(Gamepad g1, Gamepad g2) // taking advantage of copy by value to decouple
        {
            synchronized (gamepadLock) { // ensure no collisions
                localGamepad1 = g1;
                localGamepad2 = g2;
            }
            newDataReceived(); // call b/c new data to use
        }


        public void loop() {
            while (true) // perfectly valid to put infinite loop here or can run linear
            {
                double y1;
                double y2;
                boolean yButton;
                boolean lBump;
                boolean rBump;

                synchronized (gamepadLock)//get joystick values safely
                {
                    y1 = localGamepad1.left_stick_y;
                    y2 = localGamepad1.right_stick_y;
                    yButton = localGamepad1.y;
                    lBump = localGamepad1.left_bumper;
                    rBump = localGamepad1.right_bumper;

                }
                //SOS();
                if (yButton)
                {
                    if (yToggle == 1)
                    {
                        yToggle = 2;
                    }
                    else if (yToggle == 2)
                    {
                        yToggle = 1;
                    }

                }

                if (lBump) {
                    bucket.setPosition(1);
                }
                if (rBump) {
                    bucket.setPosition(0);
                }
                if (Math.abs(y1)> 0.1 && Math.abs(y2)> 0.1)
                {
                    frontRight.setPower(scaleInput(y2)/yToggle);
                    frontLeft.setPower(scaleInput(y1)/yToggle);
                    backRight.setPower(scaleInput(y2)/yToggle);
                    backLeft.setPower(scaleInput(y1)/yToggle);
                }
                else if (Math.abs(y1)> 0.1)
                {
                    frontRight.setPower(0);
                    frontLeft.setPower(scaleInput(y1)/yToggle);
                    backRight.setPower(0);
                    backLeft.setPower(scaleInput(y1)/yToggle);
                }
                else if (Math.abs(y2)> 0.1)
                {
                    frontRight.setPower(scaleInput(y2)/yToggle);
                    frontLeft.setPower(0);
                    backRight.setPower(scaleInput(y2)/yToggle);
                    backLeft.setPower(0);
                }
                else
                {
                    frontRight.setPower(0);
                    frontLeft.setPower(0);
                    backRight.setPower(0);
                    backLeft.setPower(0);
                }

                /*if (Math.abs(left) < 0.07) // deadband
                {
                    left = 0;
                }
                if (Math.abs(right) < 0.07) // deadband
                {
                    right = 0;
                }

                driveLeft.setPower(left);
                driveRight.setPower(right);

                newDataWait(5); */
            }
        }
    double scaleInput(double val)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };




        // get the corresponding index for the scaleInput array.
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
    }
    void SOS(double acc_y, double acc_z)
    {
        if (acc_y < -8.88 && acc_z > -4.14)
        {
            backLeft.setPower(-1);
            backRight.setPower(-1);
            frontLeft.setPower(-1);
            frontRight.setPower(-1);
            simpleWait(500);
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }
    }
}



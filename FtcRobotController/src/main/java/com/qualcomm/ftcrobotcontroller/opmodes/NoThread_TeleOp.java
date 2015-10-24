package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class NoThread_TeleOp extends OpMode {


    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    Servo servoBucket;
    Gamepad localGamepad1;
    Gamepad localGamepad2;
    Object gamepadLock = new Object();
    double yToggle = 1.0;
    double dScale = 0.0;

    public NoThread_TeleOp() {
        
    }

    @Override
    public void init() {
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        servoBucket = hardwareMap.servo.get("servoBucket");
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        
    }

    public void loop() {
        while (true) // perfectly valid to put infinite loop here or can run linear
        {
            double y1;
            double y2;
            boolean yButton;
            boolean lBump;
            boolean rBump;
            
            y1 = localGamepad1.left_stick_y;
            y2 = localGamepad1.right_stick_y;
            yButton = localGamepad1.y;
            lBump = localGamepad1.left_bumper;
            rBump = localGamepad1.right_bumper;
            
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
                servoBucket.setPosition(1);
            }
            else if (rBump) {
                servoBucket.setPosition(0);
            }
            else {
                servoBucket.setPosition(0.5);
            }
            if (Math.abs(y1)> 0.1 && Math.abs(y2)> 0.1)
            {
                motorFR.setPower((y2)/yToggle);
                motorFL.setPower((y1)/yToggle);
                motorBR.setPower((y2)/yToggle);
                motorBL.setPower((y1)/yToggle);
            }
            else if (Math.abs(y1)> 0.1)
            {
                motorFR.setPower(0);
                motorFL.setPower((y1)/yToggle);
                motorBR.setPower(0);
                motorBL.setPower((y1)/yToggle);
            }
            else if (Math.abs(y2)> 0.1)
            {
                motorFR.setPower((y2)/yToggle);
                motorFL.setPower(0);
                motorBR.setPower((y2)/yToggle);
                motorBL.setPower(0);
            }
            else
            {
                motorFR.setPower(0);
                motorFL.setPower(0);
                motorBR.setPower(0);
                motorBL.setPower(0);
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
    void SOS(double acc_y, double acc_z)
    {
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
}



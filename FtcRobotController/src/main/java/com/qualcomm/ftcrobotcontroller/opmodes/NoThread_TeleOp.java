package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class NoThread_TeleOp extends OpMode {

    double y1;
    double y2;
    boolean yButton;
    boolean lBump;
    boolean rBump;
    double lTrig;
    double rTrig;
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    DcMotor motorPPR;
    DcMotor motorPPL;
    //Servo servoBucket;
    Servo servoL;
    Servo servoR;

    //Gamepad localGamepad1;
    //Gamepad localGamepad2;
    Object gamepadLock = new Object();
    double yToggle = 1.0;
    double dScale = 0.0;

    public NoThread_TeleOp() {

    }

    @Override
    public void init() {
        motorBL = hardwareMap.dcMotor.get("bl");
        motorFL = hardwareMap.dcMotor.get("fl");
        motorBR = hardwareMap.dcMotor.get("br");
        motorFR = hardwareMap.dcMotor.get("fr");
        motorPPR = hardwareMap.dcMotor.get("ppr");
        motorPPL = hardwareMap.dcMotor.get("ppl");
        servoL = hardwareMap.servo.get("lServo");
        servoR = hardwareMap.servo.get("rServo");
        //
        // Bucket = hardwareMap.servo.get("servoBucket");
        /*motorFL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);*/
        
    }
    @Override
    public void loop() {

        y1 = gamepad1.left_stick_y;
        y2 = gamepad1.right_stick_y;
        yButton = gamepad1.y;
        lBump = gamepad1.left_bumper;
        lTrig = gamepad1.left_trigger;
        rTrig = gamepad1.left_trigger;




        //SOS();
        if (yButton) {
            if (yToggle == 1) {
                yToggle = 2;
            }
            else if (yToggle == 2) {
                yToggle = 1;
            }

        }


        if (lTrig > 0.1 && rTrig > 0.1) {
            //motorPPR.setPower(1.0);
            //motorPPL.setPower(-1.0);
            if (servoL.getPosition() > 0.5) {
                servoL.setPosition(0);
            }
            else {
                servoL.setPosition(1);
            }
            if (servoR.getPosition() > 0.5) {
                servoR.setPosition(0);
            }
            else {
                servoR.setPosition(1);
            }
        } else if (lTrig > 0.1) {
            if (servoL.getPosition() > 0.5) {
                servoL.setPosition(0);
            }
            else {
                servoL.setPosition(1);
            }
        } else if (rTrig > 0.1) {
            if (servoR.getPosition() > 0.5) {
                servoR.setPosition(0);
            }
            else {
                servoR.setPosition(1);
            }
        } else {

        }
        telemetry.addData("middle", "made it to joystick controls");
        if (Math.abs(y1) > 0.1 && Math.abs(y2) > 0.1) {
            motorFR.setPower((y2) / yToggle);
            motorFL.setPower(-(y1) / yToggle);
            motorBR.setPower((y2) / yToggle);
            motorBL.setPower(-(y1) / yToggle);
        } else if (Math.abs(y1) > 0.1) {
            motorFR.setPower(0);
            motorFL.setPower(-(y1) / yToggle);
            motorBR.setPower(0);
            motorBL.setPower(-(y1) / yToggle);
        } else if (Math.abs(y2) > 0.1) {
            motorFR.setPower((y2) / yToggle);
            motorFL.setPower(0);
            motorBR.setPower((y2) / yToggle);
            motorBL.setPower(0);
        } else {
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
        telemetry.addData("end", "I made it to the end of the loop");
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



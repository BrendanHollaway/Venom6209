package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class NoThread_TeleOp extends OpMode {

    double y1_1;
    double y1_2;
    double y2_1;
    double y2_2;
    boolean yButton;
    boolean lBump;
    boolean rBump;
    double lTrig;
    double rTrig;
    boolean dpadUp;
    boolean dpadDown;
    double lTrig2;
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    DcMotor motorExtendLiftR;
    DcMotor motorExtendLiftL;
    DcMotor motorRaiseLiftR;
    DcMotor motorRaiseLiftL;
    //Servo servoBucket;
    Servo servoL;
    Servo servoR;
    AdafruitIMU gyroAcc;
    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];

    //Gamepad localGamepad1;
    //Gamepad localGamepad2;
    Object gamepadLock = new Object();
    double yToggle = 1.0;
    double dScale = 0.0;

    public NoThread_TeleOp() {

    }

    public double gyroTest() {
        gyroAcc.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return yawAngle[0];
    }
    @Override
    public void init() {
        motorBL = hardwareMap.dcMotor.get("bl");
        motorFL = hardwareMap.dcMotor.get("fl");
        motorBR = hardwareMap.dcMotor.get("br");
        motorFR = hardwareMap.dcMotor.get("fr");
        motorExtendLiftR = hardwareMap.dcMotor.get("ppr");
        motorExtendLiftL = hardwareMap.dcMotor.get("ppl");
        motorRaiseLiftR = hardwareMap.dcMotor.get("ppr");
        motorRaiseLiftL = hardwareMap.dcMotor.get("ppl");
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

        y1_1 = gamepad1.left_stick_y;
        y1_2 = gamepad1.right_stick_y;
        yButton = gamepad1.y;
        lBump = gamepad1.left_bumper;
        rBump = gamepad1.right_bumper;
        lTrig = gamepad1.left_trigger;
        rTrig = gamepad1.right_trigger;
        dpadUp = gamepad1.dpad_up;
        dpadDown = gamepad1.dpad_down;
        y2_1 = gamepad2.left_stick_y;
        y2_2 = gamepad2.right_stick_y;




        //SOS();
        if (yButton) {
            if (yToggle == 1) {
                yToggle = 2;
            }
            else if (yToggle == 2) {
                yToggle = 1;
            }

        }


        if (lTrig > 0.1) {
            servoL.setPosition(Range.clip(servoL.getPosition() + 0.02, 0, 1));
        }
        if (rTrig > 0.1) {
            servoR.setPosition(Range.clip(servoR.getPosition() + 0.02, 0, 1));
        }
        if (lBump) {
            servoL.setPosition(Range.clip(servoL.getPosition() - 0.02, 0, 1));
        }
        if (rBump) {
            servoR.setPosition(Range.clip(servoR.getPosition() - 0.02, 0, 1));
        }
        if (Math.abs(y2_1) > 0.1 ) {
            if ((motorExtendLiftL.getCurrentPosition() > 1000 && y2_1 > 0) || (motorExtendLiftL.getCurrentPosition() < 10 && y2_1 < 0)) {
                motorExtendLiftL.setPower(0);
                motorExtendLiftR.setPower(0);
            }
            else {
                motorExtendLiftL.setPower(y2_1);
                motorExtendLiftR.setPower(-y2_1);
            }
        }
        if (Math.abs(y2_2) > 0.1) {
            if ((motorRaiseLiftL.getCurrentPosition() > 1000 && y2_2 > 0) || (motorRaiseLiftL.getCurrentPosition() < 10 && y2_2 < 0)) {
                motorRaiseLiftL.setPower(0);
                motorRaiseLiftR.setPower(0);
            }
            else {
                motorRaiseLiftL.setPower(y2_2);
                motorRaiseLiftR.setPower(-y2_2);
            }
        }
        if (Math.abs(y1_1) > 0.1 && Math.abs(y1_2) > 0.1) {
            motorFR.setPower(-(y1_2) / yToggle);
            motorFL.setPower((y1_1) / yToggle);
            motorBR.setPower(-(y1_2) / yToggle);
            motorBL.setPower((y1_1) / yToggle);
        } else if (Math.abs(y1_1) > 0.1) {
            motorFR.setPower(0);
            motorFL.setPower((y1_1) / yToggle);
            motorBR.setPower(0);
            motorBL.setPower((y1_1) / yToggle);
        } else if (Math.abs(y1_2) > 0.1) {
            motorFR.setPower(-(y1_2) / yToggle);
            motorFL.setPower(0);
            motorBR.setPower(-(y1_2) / yToggle);
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
        telemetry.addData("gyro yaw", gyroTest());
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



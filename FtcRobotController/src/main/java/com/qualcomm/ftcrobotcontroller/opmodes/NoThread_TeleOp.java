package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

<<<<<<< HEAD
public class NoThread_TeleOp extends LinearOpMode2{
    
=======
public class NoThread_TeleOp extends OpMode {

    double y1_1;
    double y1_2;
    double y2_1;
    double y2_2;
    boolean yButton1;
    boolean yButton2;
    boolean aButton1;
    boolean aButton2;
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
    //DcMotor motorFR;
    //DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    DcMotor motorExtendLiftR;
    DcMotor motorExtendLiftL;
    DcMotor motorRaiseLiftR;
    DcMotor motorRaiseLiftL;
    //Servo servoservoBucket;
    Servo servoL;
    Servo servoR;
    Servo servoTopRatchet;
    Servo servoBotRatchet1;
    //Servo servoBotRatchet2;
    Servo servoClimberArm;
    AdafruitIMU gyroAcc;
    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    double[] accel = new double[3];

    //Gamepad localGamepad1;
    //Gamepad localGamepad2;
    Object gamepadLock = new Object();
>>>>>>> cc691a59a8c00e515b1dc082078a6478cfdd4718
    double yToggle = 1.0;
    boolean enableSOS = true;
    int y_toggle_count=0;
    boolean SOSactive = false;
    


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
    public void runOpMode() {
        super.map();
        //servoBotRatchet1 = hardwareMap.servo.get("botRat1");
        //servoBotRatchet2 = hardwareMap.servo.get("botRat2");


        //
        // Bucket = hardwareMap.servo.get("servoBucket");
        /*motorFL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);*/
        //servoTopRatchet.setPosition(1);
        //servoClimberArm.setPosition(1);
        //servoR.setPosition(0);
        //servoL.setPosition(.7);
        //  servoBotRatchet1.setPosition(1);
    

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

        

        while(true) {

            y1_1 = gamepad1.left_stick_y;
            y1_2 = gamepad1.right_stick_y;
            yButton1 = gamepad1.y;
            yButton2 = gamepad2.y;
            aButton1 = gamepad1.a;
            aButton2 = gamepad2.a;
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
<<<<<<< HEAD
        } */
            if (aButton1) {
                //   enableSOS = !enableSOS;
=======
        }
        if (aButton1) {
            if (!enableSOS) {
                SOS_toggle_count++;
            }
            else {
                enableSOS = false;
            }
            if(SOS_toggle_count > 1)
            {
                SOS_toggle_count = 0;
                enableSOS = true;
>>>>>>> cc691a59a8c00e515b1dc082078a6478cfdd4718
            }
        /*if(gamepad1.x)
            servoTopRatchet.setPosition(1);
        if(gamepad1.b)
            servoTopRatchet.setPosition(0);*/
            SOScheck();
            telemetry.addData("SOS Check: ", enableSOS);

            // DRIVE CONTROL AND CLIMBER RELEASE


            if (lTrig1 > 0.1 ) {
                servoL.setPosition(Range.clip(servoL.getPosition() + 0.02, 0, 1));
            } else if (lBump1) {
                servoL.setPosition(Range.clip(servoL.getPosition() - 0.02, 0, 1));
            }
            if (rTrig1 > 0.1) {
                servoR.setPosition(Range.clip(servoR.getPosition() + 0.02, 0, 1));
            } else if (rBump1) {
                servoR.setPosition(Range.clip(servoR.getPosition() - 0.02, 0, 1));
            }
            if (Math.abs(y1_1) > 0.1 && Math.abs(y1_2) > 0.1 && !SOSactive) {
                motorFR.setPower(-(y1_2) / yToggle);
                motorFL.setPower((y1_1) / yToggle);
                motorBR.setPower((y1_2) / yToggle);
                motorBL.setPower(-(y1_1) / yToggle);
            } else if (Math.abs(y1_1) > 0.1 && !SOSactive) {
                motorFR.setPower(0);
                motorFL.setPower((y1_1) / yToggle);
                motorBR.setPower(0);
                motorBL.setPower(-(y1_1) / yToggle);
            } else if (Math.abs(y1_2) > 0.1 && !SOSactive) {
                motorFR.setPower(-(y1_2) / yToggle);
                motorFL.setPower(0);
                motorBR.setPower((y1_2) / yToggle);
                motorBL.setPower(0);
            } else {
                motorFR.setPower(0);
                motorFL.setPower(0);
                motorBR.setPower(0);
                motorBL.setPower(0);
            }

            // LIFT CONTROLS START HERE

            if (Math.abs(y2_1) > 0.1 ) {
                motorExtendLiftL.setPower(y2_1);
                motorExtendLiftR.setPower(-y2_1);
            } else {
                motorExtendLiftL.setPower(0);
                motorExtendLiftR.setPower(0);
            }

            if (Math.abs(y2_2) > 0.1 && !SOSactive) {
                motorRaiseLiftL.setPower(y2_2);
                motorRaiseLiftR.setPower(-y2_2);

            } else {
                motorRaiseLiftL.setPower(0);
                motorRaiseLiftR.setPower(0);
            }
            if (lBump2 )
                servoClimberArm.setPosition(0);
            else if (rBump2 )
                servoClimberArm.setPosition(1);

        /*if (lBump2) {
            if (servoArmPos == 1) {
                servoArm.setPosition(0.4);
                sleep(500);
            }
            else if (servoArmPos == 2) {
                servoArm.setPosition(0.4);
                sleep(150);
            }
            else if (servoArmPos == 3) {
                servoArm.setPosition(0.4);
                sleep(150);
            }
            else servoArm.setPosition(0.5);
        }
        else if (rBump2) {
            if (servoArmPos == 0) {
                servoArm.setPosition(0.6);
                sleep(500);
            }
            else if (servoArmPos == 1) {
                servoArm.setPosition(0.6);
                sleep(150);
            }
            else if (servoArmPos == 2) {
                servoArm.setPosition(0.6);
                sleep(150);
            }
            else servoArm.setPosition(0.5);
        }
        if (lTrig2 > 0.1) {
            servoBucketSweep.setPosition(0.5 + (lTrig2/2));
        }
        else if (rTrig2 > 0.1) {
            servoBucketSweep.setPosition(0.5 - (rTrig2/2));
        }
        if (dpadDown2) {
            servoBucketFloor.setPosition(servoBucketFloor.getPosition() + 0.05);
        }
        else if (dpadUp2) {
            servoBucketFloor.setPosition(servoBucketFloor.getPosition() - 0.05);
        }*/
            if (yButton2) {
                servoTopRatchet.setPosition(Range.clip(servoTopRatchet.getPosition() + 0.005, 0, 1));
            }
            if (aButton2) {
                servoTopRatchet.setPosition(Range.clip(servoTopRatchet.getPosition() - 0.005, 0, 1));
            }
        /*if (aButton2) {
                //servoBotRatchet1.setPosition(Range.clip(servoBotRatchet1.getPortN);
        }*/
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
    }
}




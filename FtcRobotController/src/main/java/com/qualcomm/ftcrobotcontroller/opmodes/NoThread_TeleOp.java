package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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
    //

    //Gamepad localGamepad1;
    //Gamepad localGamepad2;
    Object gamepadLock = new Object();
    double yToggle = 1.0;
    double dScale = 0.0;
    //double [] servoArmPos = new double[]{};
    int servoArmPos = 0;
    boolean servoBucketOpen = false;
    boolean enableSOS = true;
    int y_toggle_count=0;
    int SOS_toggle_count=0;


    public NoThread_TeleOp() {

    }

    public double gyroTest() {
        gyroAcc.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return yawAngle[0];
    }
    public double gyroPitch() {
        gyroAcc.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return pitchAngle[0];
    }
    public void sleep(int ms) {
        try {
            wait(ms);
        }
        catch (Exception E) {}

    }
    @Override
    public void init() {
        motorBL = hardwareMap.dcMotor.get("bl");
        //motorFL = hardwareMap.dcMotor.get("fl");
        motorBR = hardwareMap.dcMotor.get("br");
        //motorFR = hardwareMap.dcMotor.get("fr");
        motorExtendLiftR = hardwareMap.dcMotor.get("ppr");
        motorExtendLiftL = hardwareMap.dcMotor.get("ppl");
        motorRaiseLiftR = hardwareMap.dcMotor.get("rlr");
        motorRaiseLiftL = hardwareMap.dcMotor.get("rll");
        servoL = hardwareMap.servo.get("climberArm"); //dont judge
        servoR = hardwareMap.servo.get("topRat");//dont judge
        servoClimberArm = hardwareMap.servo.get("lServo");
        servoTopRatchet = hardwareMap.servo.get("rServo");
        //servoBotRatchet1 = hardwareMap.servo.get("botRat1");
        //servoBotRatchet2 = hardwareMap.servo.get("botRat2");

        long systemTime = System.nanoTime();
        try {
            gyroAcc = new AdafruitIMU(hardwareMap, "hydro"

                    //The following was required when the definition of the "I2cDevice" class was incomplete.
                    //, "cdim", 5

                    , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte)AdafruitIMU.OPERATION_MODE_IMU);
        } catch (RobotCoreException e){
        }
        gyroAcc.startIMU();
        //
        // Bucket = hardwareMap.servo.get("servoBucket");
        /*motorFL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);*/
        servoTopRatchet.setPosition(1);
        servoClimberArm.setPosition(1);
        servoR.setPosition(0);
        servoL.setPosition(.7);
      //  servoBotRatchet1.setPosition(1);
    }

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
    @Override
    public void loop() {

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
        if (yButton1) {
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
        }
        //If A button is pressed
        //If enableSOS is false, then SOS is not enabled, and thus the robot will not move backwards by itself
        // even if the robot is tilting over.
        //If it's true, then it'll save your bacon
        //if greater then 55, then SOS may need to be disabled
        if (aButton1) {
            //if SOS is not enabled
            if (!enableSOS) {
                //increase the toggle count
                SOS_toggle_count++;
            }
            //turns SOS off; SOS will not run
            else {
                enableSOS = false;
            }
            //if the count is greater than 1
            if(SOS_toggle_count > 1)
            {
                //reset toggle count and turn on SOS
                SOS_toggle_count = 0;
                enableSOS = true;
            }
        }
        /*if(gamepad1.x)
            servoTopRatchet.setPosition(1);
        if(gamepad1.b)
            servoTopRatchet.setPosition(0);*/
        if(enableSOS)
            SOScheck();
        telemetry.addData("SOS Check: ", enableSOS);

        // DRIVE CONTROL AND CLIMBER RELEASE

        if (lTrig1 > 0.1) {
            servoL.setPosition(Range.clip(servoL.getPosition() + 0.02, 0, 1));
        }
        else if (lBump1) {
            servoL.setPosition(Range.clip(servoL.getPosition() - 0.02, 0, 1));
        }
        if (rTrig1 > 0.1) {
            servoR.setPosition(Range.clip(servoR.getPosition() + 0.02, 0, 1));
        }
        else if (rBump1) {
            servoR.setPosition(Range.clip(servoR.getPosition() - 0.02, 0, 1));
        }
        if (Math.abs(y1_1) > 0.1 && Math.abs(y1_2) > 0.1) {
            //motorFR.setPower(-(y1_2) / yToggle);
            //motorFL.setPower((y1_1) / yToggle);
            motorBR.setPower((y1_2) / yToggle);
            motorBL.setPower(-(y1_1) / yToggle);
        } else if (Math.abs(y1_1) > 0.1) {
            //motorFR.setPower(0);
            //motorFL.setPower((y1_1) / yToggle);
            motorBR.setPower(0);
            motorBL.setPower(-(y1_1) / yToggle);
        } else if (Math.abs(y1_2) > 0.1) {
            //motorFR.setPower(-(y1_2) / yToggle);
            //motorFL.setPower(0);
            motorBR.setPower((y1_2) / yToggle);
            motorBL.setPower(0);
        } else {
            //motorFR.setPower(0);
            //motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);
        }

        // LIFT CONTROLS START HERE

        if (Math.abs(y2_1) > 0.1 ) {
                motorExtendLiftL.setPower(y2_1);
                motorExtendLiftR.setPower(-y2_1);
        }
        else{
            motorExtendLiftL.setPower(0);
            motorExtendLiftR.setPower(0);
        }

        if (Math.abs(y2_2) > 0.1) {
                motorRaiseLiftL.setPower(y2_2);
                motorRaiseLiftR.setPower(-y2_2);

        }
        else {
            motorRaiseLiftL.setPower(0);
            motorRaiseLiftR.setPower(0);
        }
        if (lBump2)
            servoClimberArm.setPosition(0);
        else if (rBump2)
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
            /*int hold = 0;
            if(yButton2) {
                hold++;
            }
            if (hold > 4) */
                servoTopRatchet.setPosition(0);
        }
        if (aButton2) {
            int hold = 0;
            while (yButton2) {
                hold++;
            }
            if (hold > 4) {
                //servoBotRatchet1.setPosition(0);
            }
        }
        telemetry.addData("gyro yaw", gyroTest());
        telemetry.addData("gyro pitch", gyroPitch());
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
    void SOScheck()
    {
        gyroAcc.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        if (pitchAngle[0] > 55 && enableSOS)
        {
            motorBL.setPower(-1);
            motorBR.setPower(1);                    //If the robot is flipping over, then driver control is
            //motorFL.setPower(-1);                 //taken away and motors run backwards to stabilize it
            //motorFR.setPower(-1);
            try {
                wait(500);
            }
            catch (Exception E){}
            motorBL.setPower(0);
            motorBR.setPower(0);
            //motorFL.setPower(0);
            //motorFR.setPower(0);
        }
    }
}



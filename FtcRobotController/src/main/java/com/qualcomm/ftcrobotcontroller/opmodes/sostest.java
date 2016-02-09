package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class sostest extends OpMode {

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
        //servoClimberArm = hardwareMap.servo.get("lServo");
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
            telemetry.addData("IMU IS DEAD: ", "IT THREW AN ERROR");
        }
        gyroAcc.startIMU();
    }

    @Override
    public void loop() {
        gyroAcc.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        if (pitchAngle[0] < -10)
        {
            motorBL.setPower(-1);
            motorBR.setPower(1);                    //If the robot is flipping over, then driver control is
            //motorFL.setPower(0);
            //motorFR.setPower(0);
        }    }
}
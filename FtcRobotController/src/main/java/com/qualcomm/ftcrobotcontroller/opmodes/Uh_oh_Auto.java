package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by viperbots on 12/4/2015.
 */
public class Uh_oh_Auto extends LinearOpMode {
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
    AutonomousSegments auto;

    public void runOpMode() throws InterruptedException
    {
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
            telemetry.addData("IMU IS DEAD: ", "IT THREW AN ERROR");
        }
        gyroAcc.startIMU();
        waitForStart();
        auto = new AutonomousSegments(motorBL, motorBR, gyroAcc);
        // Bucket = hardwareMap.servo.get("servoBucket");
        /*motorFL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);*/
        servoTopRatchet.setPosition(1);
        servoClimberArm.setPosition(1);
        servoR.setPosition(0);
        servoL.setPosition(.7);
        motorBL.setPower(1);
        motorBR.setPower(1);
        try {
            sleep(15000);
        }
        catch(Exception E)
        {}
        motorBL.setPower(0);
        motorBR.setPower(0);

    }
}

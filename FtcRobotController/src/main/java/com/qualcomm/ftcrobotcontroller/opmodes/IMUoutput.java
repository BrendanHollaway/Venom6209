package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
/**
 * Created by viperbots on 11/14/2015.
 */
public class IMUoutput extends OpMode{
    AdafruitIMU gyroAcc;
    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];

    public double gyroTest() {
        gyroAcc.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return yawAngle[0];
    }
    public void init() {
        long systemTime = System.nanoTime();
        try {
            gyroAcc = new AdafruitIMU(hardwareMap, "IMU"

                    //The following was required when the definition of the "I2cDevice" class was incomplete.
                    //, "cdim", 5

                    , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte)AdafruitIMU.OPERATION_MODE_IMU);
        } catch (RobotCoreException e){
        }

    }
    @Override
    public void loop() {
        gyroAcc.startIMU();

        gyroTest();
        telemetry.addData("gyro yaw", gyroTest());
    }
}

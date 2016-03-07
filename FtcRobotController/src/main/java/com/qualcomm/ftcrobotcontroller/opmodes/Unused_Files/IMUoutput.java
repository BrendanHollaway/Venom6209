package com.qualcomm.ftcrobotcontroller.opmodes.Unused_Files;

import android.util.Log;

import com.qualcomm.ftcrobotcontroller.opmodes.AdafruitIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;

import java.util.Timer;

/**
 * Created by viperbots on 11/14/2015.
 */
public class IMUoutput extends OpMode{
    AdafruitIMU gyroAcc;
    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    double[] accel = new double[3];

    public double yawTest() {
        gyroAcc.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return yawAngle[0];
    }
    public void init() {
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

    }
    double velocity_x = 0;
    @Override
    public void loop() {

        telemetry.addData("gyro yaw", yawTest());
        gyroAcc.getAccel(accel);
        try{
            telemetry.addData("acc_X: ", accel[0]);
            telemetry.addData("acc_Y: ", accel[1]);
            telemetry.addData("acc_Z: ", accel[2]);
            velocity_x += accel[0];
        }
        catch(Exception e)
        {
            telemetry.addData("Error: ", e.getMessage());
        }
    }
}

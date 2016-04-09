package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Brendan Hollaway and Bo Deng on 3/13/2016.
 */
public class SegwayBot extends OpMode {
    DcMotor motorL;
    DcMotor motorR;
    AdafruitIMU IMU;
    double PID_change;
    private final double kP = 0.03;
    private final double kI = 0.0;
    private final double kD = .65;

    @Override
    public void init() {
        motorL = hardwareMap.dcMotor.get("motorL");
        motorR = hardwareMap.dcMotor.get("motorR");
        motorR.setDirection(DcMotor.Direction.REVERSE);
        try {
            IMU = new AdafruitIMU(hardwareMap, "IMU"

                    //The following was required when the definition of the "I2cDevice" class was incomplete.
                    //, "cdim", 5

                    , (byte) (AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte) AdafruitIMU.OPERATION_MODE_IMU);
            telemetry.addData("IMU IS ALIVE: ", "GOOOOOOOOOOOOD!");
        } catch (RobotCoreException e) {
            telemetry.addData("IMU IS DEAD: ", "IT THREW AN ERROR");
        }

        IMU.startIMU();
        IMU.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        telemetry.addData("IMU Initialized? ", IMU.offsetsInitialized);
        while(!IMU.offsetsInitialized)
            IMU.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        telemetry.addData("IMU Initialized? ", IMU.offsetsInitialized);
    }
       public void loop()
        {
            telemetry.addData("gyro_Yaw: ", getGyroYaw());
            telemetry.addData("gyro Pitch: ", getGyroPitch());
            telemetry.addData("gyro Roll: ", getGyroRoll());
            if(gamepad1.dpad_up)
                PID_change = Range.clip(get_PID_Pitch(5, kP, kI, kD), -1, 1);
            else if(gamepad1.dpad_down)
                PID_change = Range.clip(get_PID_Pitch(-5, kP, kI, kD), -1, 1);
            else
                PID_change = Range.clip(get_PID_Pitch(0, kP, kI, kD), -1, 1);
            if (Math.abs(getGyroPitch()) >= 2.5) {
                motorL.setPower(PID_change);
                motorR.setPower(PID_change);
            }
            DbgLog.error("PID Change: " + PID_change);
        }


    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    double curr_heading = 0;
    double curr_PID = 0;
    double prevError;
    double error = 0;
    double iError = 0;
    double dError;
    double time = getRuntime();
    double dt;
    public double get_PID(double target_heading, double gyro, double kP, double kI, double kD)
    {

        dt = getRuntime() - time;
        time = getRuntime();
        prevError = error;
        if(Math.abs(target_heading - gyro + 360) < Math.abs(target_heading - gyro))
            error = target_heading - gyro + 360;
        else
            error = target_heading - gyro;
        dError = (error - prevError) / dt;
        //make this a reimann right sum if needed to improve speed at the cost of accuracy
        iError = Range.clip(iError + 0.5 * (prevError + error) * dt, -125, 125) * 0.99; // a trapezoidal approximation of the integral.
        return kP * error + kD * dError + kI * iError;
    }
    public double get_PID_Pitch(double target_pitch, double kP, double kI, double kD)
    {
        double gyro = getGyroPitch();
        dt = getRuntime() - time;
        time = getRuntime();
        prevError = error;
        if(Math.abs(target_pitch - gyro + 360) < Math.abs(target_pitch - gyro))
            error = target_pitch - gyro + 360;
        else
            error = target_pitch - gyro;
        dError = (error - prevError) / dt;
        //make this a reimann right sum if needed to improve speed at the cost of accuracy
        iError = Range.clip(iError + 0.5 * (prevError + error) * dt, -125, 125) * 0.99; // a trapezoidal approximation of the integral.
        return kP * error + kD * dError + kI * iError;
    }
    public double getGyroYaw() {
        if(IMU == null) {
            DbgLog.error(" IMU is null");
            return -1;
        }
        IMU.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return yawAngle[0];
    }
    public double getGyroPitch() {
        if(IMU == null) {
            DbgLog.error(" IMU is null");
            return -1;
        }
        IMU.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return pitchAngle[0];
    }
    public double getGyroRoll() {
        if(IMU == null) {
            DbgLog.error(" IMU is null");
            return -1;
        }
        IMU.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return rollAngle[0];
    }
}

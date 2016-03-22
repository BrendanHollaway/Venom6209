package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Brendan Hollaway on 3/15/2016.
 */
public class PID_Tuner extends LinearOpModeCV {
    AutonomousSegments auto = new AutonomousSegments(telemetry, this);

    public void runOpMode() throws InterruptedException
    {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        try {
            IMU = new AdafruitIMU(hardwareMap, "IMU"

                    //The following was required when the definition of the "I2cDevice" class was incomplete.
                    //, "cdim", 5

                    , (byte) (AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte) AdafruitIMU.OPERATION_MODE_IMU);
            telemetry.addData("IMU IS ALIVE: ", "NO ERRORS!");
        } catch (RobotCoreException e) {
            telemetry.addData("IMU IS DEAD: ", "IT THREW AN ERROR");
            DbgLog.logStacktrace(e);
        }
        auto.tune_PID();
    }
}

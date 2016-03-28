package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Mat;

/**
 * Created by Brendan Hollaway on 3/22/2016.
 */
public class VoltageTester extends LinearOpModeCV {
    double motorFR_offset = 0;
    double motorFL_offset = 0;
    double motorBR_offset = 0;
    double motorBL_offset = 0;
    public void runOpMode() throws InterruptedException {
        waitForStart();
        resetStartTime();
        double end_time = System.currentTimeMillis() + 1000 * 1E3;
        int count = 0;
        while(getRuntime() < 1000)
        {
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double startTime = getRuntime();
            while(getRuntime() < count * 5)
                setMotors(1);
            DbgLog.msg(String.format("oldVoltage: %.2f, runTime: %.2f, encoderTicks: %.2f", voltage, getRuntime() - startTime, getMid2()));
            updateOffsets();
            while(getRuntime() < count * 5 + 1);
        }
    }
    public void halt() throws InterruptedException{
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }
    public void updateOffsets()
    {
        motorFR_offset = motorFR.getCurrentPosition();
        motorFL_offset = motorFL.getCurrentPosition();
        motorBR_offset = motorBR.getCurrentPosition();
        motorBL_offset = motorBL.getCurrentPosition();
    }
    public double getMid2()
    {
        double abs_FR = Math.abs(motorFR.getCurrentPosition() - motorFR_offset);
        double abs_FL = Math.abs(motorFL.getCurrentPosition() - motorFL_offset);
        double abs_BR = Math.abs(motorBR.getCurrentPosition() - motorBR_offset);
        double abs_BL = Math.abs(motorBL.getCurrentPosition() - motorBL_offset);
        double max = Math.max(abs_FR, Math.max(abs_FL, Math.max(abs_BR, abs_BL)));
        double min = Math.min(abs_FR, Math.min(abs_FL, Math.min(abs_BR, abs_BL)));
        return abs_BL + abs_BR + abs_FL + abs_FR - max - min;
    }
    public void setMotors(double speed)
    {
        motorFR.setPower(speed);
        motorFL.setPower(speed);
        motorBR.setPower(speed);
        motorBL.setPower(speed);
    }
}

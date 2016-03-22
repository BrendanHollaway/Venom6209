package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Brendan Hollaway on 3/13/2016.
 */
public class SegwayBot extends LinearOpModeCV {
    DcMotor motorL;
    DcMotor motorR;
    AdafruitIMU IMU;
    AutonomousSegments auto = new AutonomousSegments(telemetry, this);
    double PID_change;
    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0;

    public void runOpMode() throws InterruptedException
    {
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
            telemetry.addData("IMU IS ALIVE: ", "NO ERRORS!");
        } catch (RobotCoreException e) {
            telemetry.addData("IMU IS DEAD: ", "IT THREW AN ERROR");
        }
        while(!opModeIsActive());
        while(opModeIsActive())
        {
            if(gamepad1.dpad_up)
                PID_change = Range.clip(auto.get_PID_Pitch(-40, kP, kI, kD), -1, 1);
            else if(gamepad1.dpad_down)
                PID_change = Range.clip(auto.get_PID_Pitch(-50, kP, kI, kD), -1, 1);
            else
                PID_change = Range.clip(auto.get_PID_Pitch(-45, kP, kI, kD), -1, 1);
            motorL.setPower(PID_change);
            motorR.setPower(PID_change);
            waitOneFullHardwareCycle();
        }
    }
}

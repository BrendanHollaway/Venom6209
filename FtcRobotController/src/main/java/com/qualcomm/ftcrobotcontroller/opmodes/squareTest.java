package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by viperbots on 10/12/2015.
 */
public class squareTest extends LinearOpMode {
    DcMotor motorFL;
    DcMotor motorBL;
    DcMotor motorFR;
    DcMotor motorBR;
    AutonomousSegments auto;

    public void runOpMode() throws InterruptedException {

        motorFL = hardwareMap.dcMotor.get("fl");
        motorBL = hardwareMap.dcMotor.get("bl");
        motorFR = hardwareMap.dcMotor.get("fr");
        motorBR = hardwareMap.dcMotor.get("br");
        //auto = new AutonomousSegments(motorFL, motorBL, motorBR, motorFR);

        try {
            waitForStart();
        }
        catch(Exception E)
        {
            telemetry.addData("Interupted", E);
        }
        auto.ssleep(10000);
        motorFL.setPower(-1);
        motorBL.setPower(-1);                 //takes sign of position, so sign of speed does not matter
        motorFR.setPower(1);
        motorBR.setPower(1);
        auto.ssleep(15000);
        auto.halt();
    }
}

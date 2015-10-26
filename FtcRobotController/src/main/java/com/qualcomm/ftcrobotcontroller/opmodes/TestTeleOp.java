package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by viperbots on 10/26/2015.
 */
public class TestTeleOp extends OpMode{

    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    DcMotor motorPPR;
    DcMotor motorPPL;
    public void init(){
        motorBL = hardwareMap.dcMotor.get("bl");
        motorFL = hardwareMap.dcMotor.get("fl");
        motorBR = hardwareMap.dcMotor.get("br");
        motorFR = hardwareMap.dcMotor.get("fr");
        motorPPR = hardwareMap.dcMotor.get("ppr");
        motorPPL = hardwareMap.dcMotor.get("ppl");
    }
    public void loop(){
        if(gamepad1.a)
            motorFR.setPower(1.0); //need to reverse
        else
            motorFR.setPower(0.0);
        if(gamepad1.b)
            motorFL.setPower(1.0);
        else
            motorFL.setPower(0.0);
        if(gamepad1.x)
            motorBR.setPower(1.0);
        else
            motorBR.setPower(0.0);
        if(gamepad1.y)
            motorBL.setPower(1.0);
        else
            motorBL.setPower(0.0);
        if(gamepad1.left_bumper)
            motorPPL.setPower(1.0);
        else
            motorPPL.setPower(0.0);
        if(gamepad1.right_bumper)
            motorPPR.setPower(1.0);
        else
            motorPPR.setPower(0.0);
    }
}

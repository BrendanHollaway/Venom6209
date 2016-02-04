package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by viperbots on 10/26/2015.
 */
public class TestTeleOp extends OpMode{

    DcMotor motorFR;
    DcMotor motorFL;
    //DcMotor motorBR;
    //DcMotor motorBL;
    DcMotor liftL;
    DcMotor liftR;
    public void init(){
        //motorBL = hardwareMap.dcMotor.get("bl");
        motorFL = hardwareMap.dcMotor.get("fl");
        //motorBR = hardwareMap.dcMotor.get("br");
        motorFR = hardwareMap.dcMotor.get("fr");
        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");
    }
    public void loop(){
        if(Math.abs(gamepad2.left_stick_y) > 0.1 && Math.abs(gamepad2.right_stick_y) > 0.1)
        {
            liftL.setPower(gamepad2.right_stick_y);
            liftR.setPower(gamepad2.left_stick_y);
        }
        else if(Math.abs(gamepad2.right_stick_y) > 0.1)
        {
            liftL.setPower(gamepad2.right_stick_y);
            liftR.setPower(0);
        }
        else if(Math.abs(gamepad2.left_stick_y) > 0.1)
        {
            liftL.setPower(0);
            liftR.setPower(gamepad2.left_stick_y);
        }
        else
        {
            liftL.setPower(0);
            liftR.setPower(0);
        }
        if (Math.abs(gamepad1.left_stick_y) > 0.1 && Math.abs(gamepad1.right_stick_y) > 0.1) {
            motorFR.setPower(gamepad1.left_stick_y);
            motorFL.setPower(-gamepad1.right_stick_y);
            //motorBR.setPower(gamepad1.left_stick_y);
            //motorBL.setPower(-gamepad1.right_stick_y);
        } else if (Math.abs(gamepad1.left_stick_y) > 0.1) {
            motorFR.setPower(gamepad1.left_stick_y);
            motorFL.setPower(0);
            //motorBR.setPower(gamepad1.left_stick_y);
            //motorBL.setPower(0);
        } else if (Math.abs(gamepad1.right_stick_y) > 0.1) {
            motorFR.setPower(0);
            motorFL.setPower(-gamepad1.right_stick_y);
            //motorBR.setPower(0);
            //motorBL.setPower(-gamepad1.right_stick_y);
        } else if(gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1)
        {
            //motorBR.setPower(gamepad1.right_trigger > 0.1? gamepad1.right_trigger : 0);
            //motorBL.setPower(gamepad1.left_trigger > 0.1? -gamepad1.left_trigger : 0);
        }
        else {
            motorFR.setPower(0);
            motorFL.setPower(0);
            //motorBR.setPower(0);
            //motorBL.setPower(0);
        }
            
        /*if(gamepad1.a)
            motorFR.setPower(1.0); //need to reverse
        else
            motorFR.setPower(0.0);
        if(gamepad1.b)
            motorFL.setPower(1.0);
        else
            motorFL.setPower(0.0);
        /*if(gamepad1.x)
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
            motorPPR.setPower(0.0);*/
    }
}

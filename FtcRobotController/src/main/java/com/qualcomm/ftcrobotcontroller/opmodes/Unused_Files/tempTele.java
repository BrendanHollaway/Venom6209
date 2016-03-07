package com.qualcomm.ftcrobotcontroller.opmodes.Unused_Files;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by viperbots on 12/23/2015.
 */
public class tempTele extends OpMode {
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    DcMotor pulleyL;
    DcMotor pulleyR;
    Servo servoL;
    Servo servoR;
    double y1_1;
    double y1_2;
    double y2_1;
    double y2_2;
    public void init()
    {
        motorBL = hardwareMap.dcMotor.get("bl");
        motorFL = hardwareMap.dcMotor.get("fl");
        motorBR = hardwareMap.dcMotor.get("br");
        motorFR = hardwareMap.dcMotor.get("fr");
        pulleyL = hardwareMap.dcMotor.get("pl");
        pulleyR = hardwareMap.dcMotor.get("pr");
        pulleyL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        servoL = hardwareMap.servo.get("climberArm");
        servoR = hardwareMap.servo.get("topRat");
    }
    public void loop()
    {
        y1_1 = gamepad1.left_stick_y;
        y1_2 = gamepad1.right_stick_y;
        y2_1 = gamepad2.left_stick_y;
        y2_2 = gamepad2.right_stick_y;
        if(gamepad1.a == true)
        {
            servoR.setPosition(Range.clip(servoR.getPosition() + 0.005, 0, 1));
        }
        if(gamepad1.b == true)
        {
            servoL.setPosition(Range.clip(servoL.getPosition() + 0.005, 0, 1));
        }

        if (Math.abs(y1_1) > 0.1 && Math.abs(y1_2) > 0) {
            motorFR.setPower(y1_2);
            motorFL.setPower(y1_1);
            motorBR.setPower(y1_2);
            motorBL.setPower(y1_1);
        } else if (Math.abs(y1_1) > 0.1) {
            motorFR.setPower(0);
            motorFL.setPower(y1_1);
            motorBR.setPower(0);
            motorBL.setPower(y1_1);
        } else if (Math.abs(y1_2) > 0.1) {
            motorFR.setPower(y1_2);
            motorFL.setPower(0);
            motorBR.setPower(y1_2);
            motorBL.setPower(0);
        } else {
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);
        }
        if(Math.abs(y2_1) > 0)
        {
            pulleyL.setPower(y2_1);
            pulleyR.setPower(y2_1);
        }
        else if(gamepad2.right_bumper)
            pulleyR.setPower(-0.25);
        else if(gamepad2.right_trigger > 0.1)
            pulleyR.setPower(0.25);
        else if(gamepad2.left_bumper)
            pulleyL.setPower(-0.25);
        else if(gamepad2.left_trigger > 0.1)
            pulleyL.setPower(0.25);
        else
        {
            pulleyL.setPower(0);
            pulleyR.setPower(0);
        }
        /*if(gamepad1.dpad_up)
            motorFR.setPower(1);
        if(gamepad1.dpad_down)
            motorFL.setPower(1); // needs neg
        if(gamepad1.dpad_left)
            motorBL.setPower(1); // needs neg
        if(gamepad1.dpad_right)
            motorBR.setPower(1);
        */
        telemetry.addData("servoR: ", servoR.getPosition());
        telemetry.addData("servoL: ", servoL.getPosition());
    }
}
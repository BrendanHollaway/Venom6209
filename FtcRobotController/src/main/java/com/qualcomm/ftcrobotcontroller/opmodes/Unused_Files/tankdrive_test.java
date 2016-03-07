package com.qualcomm.ftcrobotcontroller.opmodes.Unused_Files;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class tankdrive_test extends OpMode {
    DcMotor motorBR;
    DcMotor motorBL;
    double y1_1;
    double y1_2;
    
    public void init()
    {
        motorBL = hardwareMap.dcMotor.get("bl");
        //motorFL = hardwareMap.dcMotor.get("fl");
        motorBR = hardwareMap.dcMotor.get("br");
        //motorFR = hardwareMap.dcMotor.get("fr");
    }
    public void loop() {
        y1_1 = gamepad1.left_stick_y;
        y1_2 = gamepad1.right_stick_y;
        
        if (Math.abs(y1_1) > 0.1 && Math.abs(y1_2) > 0.1) {
            //motorFR.setPower(-(y1_2));
            //motorFL.setPower((y1_1));
            motorBR.setPower((y1_2));
            motorBL.setPower(-(y1_1));
        } else if (Math.abs(y1_1) > 0.1) {
            //motorFR.setPower(0);
            //motorFL.setPower((y1_1));
            motorBR.setPower(0);
            motorBL.setPower(-(y1_1));
        } else if (Math.abs(y1_2) > 0.1) {
            //motorFR.setPower(-(y1_2));
            //motorFL.setPower(0);
            motorBR.setPower((y1_2));
            motorBL.setPower(0);
        } else {
            //motorFR.setPower(0);
            //motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);
        }
    }
    
}

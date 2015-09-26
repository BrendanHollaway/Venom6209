/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
public class NotATrollBotTeleOp extends OpMode {
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
    Servo armHorizontal;
    Servo armVertical;
    float armV = 0;
    float armH = 0;

    public NotATrollBotTeleOp() {
    }

    /*
		* Code to run when the op mode is first enabled goes here
		* @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
				*/
    @Override
    public void init() {
        motorBL = hardwareMap.dcMotor.get("motor_1");
        motorBR = hardwareMap.dcMotor.get("motor_2");
        motorFR = hardwareMap.dcMotor.get("motor_3");
        motorFL = hardwareMap.dcMotor.get("motor_4");
        armHorizontal = hardwareMap.servo.get("servo_2");
        armVertical = hardwareMap.servo.get("servo_1");
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        float y1_1 = gamepad1.left_stick_y;
        float x1_2 = gamepad1.left_stick_x;
        float x1_1 = gamepad1.right_stick_x;
        float y1_2 = -gamepad1.right_stick_y; // reverse right wheel
        //float trigL = gamepad1.left_trigger;
        //float trigR = gamepad1.right_trigger;
        boolean bumpL = gamepad1.left_bumper;
        boolean bumpR = gamepad1.right_bumper;
        float y2_1 = gamepad2.left_stick_y;
        float x2_2 = gamepad2.left_stick_x;
        float x2_1 = gamepad2.right_stick_x;
        float y2_2 = gamepad2.right_stick_y;
        armVertical.setPosition(0.5);
        armHorizontal.setPosition(0.5);
		/*if(y1_1 > 0.1 || y1_1 < -0.1 || x1 > 0.1 || x1 < -0.1 || y1_2 > 0.1 || y1_2 < -0.1) //checks if sticks are being moved
		{
			if(y1_1 > 0.1 || y1_1 < -0.1) // Up/Down
			{
				motorFL.setPower(-y1_1); //neg
				motorBL.setPower(-y1_1); //neg
				motorFR.setPower(y1_1); //pos
				motorBR.setPower(y1_1); //pos
			}
			else if(x1 > 0.1 || x1 < -0.1) // Right/Left
			{
				motorFL.setPower(-x1); //neg
				motorBL.setPower(x1); //pos
				motorFR.setPower(-x1); //neg
				motorBR.setPower(x1); //pos
			}
			else if(y1_2 > 0.1 || y1_2 < -0.1) //rotate
			{
				motorFL.setPower(y1_2); //pos
				motorBL.setPower(y1_2); //pos
				motorFR.setPower(y1_2); //pos
				motorBR.setPower(y1_2); //pos
			{
		}*/

        if (bumpL)
        {
            armVertical.setPosition(1.0);
        }
        else if (bumpR)

        {
            armVertical.setPosition(0.0);
        }
        else
        {
            armVertical.setPosition(0.5);
        }
        if (Math.abs(y1_1) > 0.1 && Math.abs(y1_2) > 0.1) {
            motorBL.setPower(y1_1);
            motorBR.setPower(y1_2);
            motorFL.setPower(y1_1);
            motorBR.setPower(y1_2);
        } else if (Math.abs(y1_1) > 0.1) {
            motorBL.setPower(y1_1);
            motorFL.setPower(y1_1);
            motorBR.setPower(0);
            motorFR.setPower(0);
        } else if (Math.abs(y1_2) > 0.1) {
            motorBR.setPower(y1_2);
            motorFR.setPower(y1_2);
            motorBL.setPower(0);
            motorFL.setPower(0);
        } else //individual motor control
        {
            motorFL.setPower(0.0);
            motorBL.setPower(0.0);
            motorFR.setPower(0.0);
            motorBR.setPower(0.0);
			/*if (trigL > 0.1 || trigR > 0.1 || bumpR || bumpL) {
				if (bumpL) //motorBL control
				{
					motorBL.setPower(1.0);
				}
				if (bumpR) //motorBR control
				{
					motorBR.setPower(1.0);
				}
				if (trigR > 0.1) //motorFR control
				{
					//motorFR.setPower(1.0);
				}
				if (trigL > 0.1) //motorFL control
				{
					//motorFL.setPower(1.0);
				}
			} else {
				//motorFL.setPower(0.0);
				motorBL.setPower(0.0);
				//motorFR.setPower(0.0);
				motorBR.setPower(0.0);
			}*/
        //}
       /* if (Math.abs(y2_1) > 0.1 && Math.abs(x2_2) > 0.1)
        {
            armV += y2_1/80.0;
            armV = Range.clip(armV, 0, 1);
            armVertical.setPosition(armV);
            armH += x2_2/80.0;
            armH = Range.clip(armH, 0, 1);
            armHorizontal.setPosition(armH);
        }
        else if (Math.abs(y2_1) > 0.1)
        {
            armV += y2_1/80.0;
            armV = Range.clip(armV, 0, 1);
            armVertical.setPosition(armV);
        }
        else if (Math.abs(x2_2) > 0.1)
        {
            armH += x2_2/80.0;
            armH = Range.clip(armH, 0, 1);
            armHorizontal.setPosition(armH);
        }*/
    }
    }



        /*telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("clawL", "clawL:  " + String.format("%.2f", clawL));
        telemetry.addData("clawR", "clawR:  " + String.format("%.2f", clawR));*/


    public void stop() {

    }
}
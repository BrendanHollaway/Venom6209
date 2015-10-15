/* Copyright (c) 2015 Qualcomm Technologies Inc

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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A simple example of a linear op mode that will approach an IR beacon
 */
public class TrashDashAutonomous extends LinearOpMode {

   DcMotor motorBL;
   DcMotor motorBR;
   DcMotor motorFL;
   DcMotor motorFR;
   DcMotor motorPulley;
   Servo clawLeft;
   Servo clawRight;
  final  double CLAW_LEFT_OPEN  = 0.20;
  final  double CLAW_LEFT_CLOSED  = 0.7;
  final  double CLAW_RIGHT_OPEN  = 0.20;
  final  double CLAW_RIGHT_CLOSED  = 0.7;

  public  void move(int ms, double speed) {
      //double encoder = ms * 280 / (3 * 3.14);
      motorBL.setPower(speed);
      motorBR.setPower(speed);
      motorFL.setPower(speed);
      motorFR.setPower(speed);
      //ssleep(ms);
      halt();
  }
  public  void halt() {
      motorBL.setPower(0);
      motorBR.setPower(0);
      motorFL.setPower(0);
      motorFR.setPower(0);
      //sleep(100);
  }
  public  void left(int ms, double speed) {
      motorBR.setPower(speed);
      motorFR.setPower(speed);
      motorBL.setPower(0);
      motorFL.setPower(0);
      //sleep(ms);
      halt();
  }
  public  void right(int ms, double speed) {
      motorBR.setPower(0);
      motorFR.setPower(0);
      motorBL.setPower(speed);
      motorFL.setPower(speed);
      //sleep(ms);
      halt();
  }
    public  void openClaw(int ms /* wait for claw to move */) {
        clawLeft.setPosition(CLAW_LEFT_OPEN);
        clawRight.setPosition(CLAW_RIGHT_OPEN);
        //sleep(ms);
    }
    public  void closeClaw(int ms) {
        clawLeft.setPosition(CLAW_LEFT_CLOSED);
        clawRight.setPosition(CLAW_RIGHT_CLOSED);
        //sleep(ms);
    }


  @Override
  public void runOpMode() throws InterruptedException {

    // set up the hardware devices we are going to use
    motorBL = hardwareMap.dcMotor.get("motor_1");
    motorBR = hardwareMap.dcMotor.get("motor_2");
    motorFR = hardwareMap.dcMotor.get("motor_3");
    motorFL = hardwareMap.dcMotor.get("motor_4");
    motorPulley = hardwareMap.dcMotor.get("motor_5");
    clawLeft = hardwareMap.servo.get("servo_1");
    clawRight = hardwareMap.servo.get("servo_2");



    // wait for the start button to be pressed
      waitForStart();
      {
          openClaw(0);
          move(1000,1);
          closeClaw(1000);
          move(1000,-1);
          right(250,1);
          move(1000,1);
          openClaw(500);
          move(1000,-1);
          right(250,-1);
      }
    // wait for the IR Seeker to detect a signal
    /*while (!irSeeker.signalDetected()) {
      sleep(1000);
    }

    if (irSeeker.getAngle() < 0) {
      // if the signal is to the left move left
      motorRight.setPower(MOTOR_POWER);
      motorLeft.setPower(-MOTOR_POWER);
    } else if (irSeeker.getAngle() > 0) {
      // if the signal is to the right move right
      motorRight.setPower(-MOTOR_POWER);
      motorLeft.setPower(MOTOR_POWER);
    }

    // wait for the robot to center on the beacon
    while (irSeeker.getAngle() != 0) {
      waitOneHardwareCycle();
    }

    // now approach the beacon
    motorRight.setPower(MOTOR_POWER);
    motorLeft.setPower(MOTOR_POWER);

    // wait until we are close enough
    while (irSeeker.getStrength() < HOLD_IR_SIGNAL_STRENGTH) {
      waitOneHardwareCycle();
    }

    // halt the motors
    motorRight.setPower(0);
    motorLeft.setPower(0);*/
  }
    public void ssleep(long ms) throws InterruptedException
    {
        try{
            sleep(ms);
        }
        catch(Exception E)
        {
        }
    }
}

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs.BufferSupportedRunnable;
import com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs.BufferedMotor;
import com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs.BufferedServo;
import com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs.BufferedSmartMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by viperbots on 9/21/2015.
 */
public class Arm_TeleOp extends BufferSupportedRunnable {

    BufferedSmartMotor armMotor;
    BufferedServo clawServo;
    Gamepad localGamepad1;
    Gamepad localGamepad2;
    Object gamepadLock = new Object();

    public Arm_TeleOp(HardwareMap map) {
        super(map);
        armMotor = BufferedSmartMotor.createBufferedSmartMotor(map,"Arm Motor"); // change names as necessary in ""
        clawServo = BufferedServo.createBufferedMotor(map, "Claw Servo");

        //setup motor for usage
        armMotor.attachEncoder();
        armMotor.setBufferMode(BufferedMotor.BUFFER_METHOD.POWER_AND_ENCODER);
        armMotor.enableSmartTargetSeeking(); // tell motor we want to use internal pid loop to seek
        armMotor.hardwareResetBothAttachedEncodersValues(); // call once per motor on a controller to reset
        armMotor.resetEncoderValue();
    }

    @Override
    public void newDataReceived() { // NOTE: Not called from this thread.  Called from opMode thread
        super.newDataReceived();
    }

    public void passNewJoystickInfo(Gamepad g1,Gamepad g2) // taking advantage of copy by value to decouple
    {
        synchronized (gamepadLock) { // ensure no collisions
            localGamepad1 = g1;
            localGamepad2 = g2;
        }
        newDataReceived(); // call b/c new data to use
    }

    @Override
    public void run() {
        boolean moveToTarget = false;
        while (true) // perfectly valid to put infinite loop here or can run linear
        {
            double armControl;
            double clawControl;
            int target=0;
            synchronized (gamepadLock)//get joystick values safely
            {
                armControl = localGamepad1.left_stick_y;

                // open close claw control
                clawControl = (localGamepad1.b)?1:(localGamepad1.a)?0:0.5; // all cw if button B , all ccw if button A, centered otherwise
                if (localGamepad1.y)
                {
                    moveToTarget=true;
                    target=1000;
                }
                else if (localGamepad1.y)
                {
                    moveToTarget=true;
                    target=2000;
                }
                else
                {
                    moveToTarget=false;
                }
            }

            clawServo.setPosition(clawControl);

            //logic for switching between manual control and PID position seeking provided by BufferedSmartMotor class
            armMotor.setAutoUpdatePower(moveToTarget); // if false is passed in, PID loop runs but doesn't modify the motor power
            armMotor.setPower(armControl); // call this before updatePIDLoop()
            armMotor.setSmartTargetPosition(target); // allow PID to update power automatically

            armMotor.updatePIDLoop(); // always call last on smart motor

            newDataWait(5);
        }
    }
}

package com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by jspspike on 8/26/15.
 */
public class RevTrollbot_Drive_BufferedRunnable extends BufferSupportedRunnable{
    BufferedInterpolatedMotor motorBL;
    BufferedInterpolatedMotor motorBR;
    BufferedInterpolatedMotor motorFL;
    BufferedInterpolatedMotor motorFR;


    Gamepad localGamepad1;
    Gamepad localGamepad2;

    Object gamepadLock = new Object();

    public RevTrollbot_Drive_BufferedRunnable(HardwareMap map) {
        super(map);

        motorBL = BufferedInterpolatedMotor.createBufferedInterpolatedMotor(map, "motor_1");
        motorBR = BufferedInterpolatedMotor.createBufferedInterpolatedMotor(map, "motor_2");
        motorFL = BufferedInterpolatedMotor.createBufferedInterpolatedMotor(map, "motor_3");
        motorFR = BufferedInterpolatedMotor.createBufferedInterpolatedMotor(map, "motor_4");
    }

    public void newDataReceived() {super.newDataReceived();}

    public void passNewJoystickInfo(Gamepad g1, Gamepad g2) {
        synchronized (gamepadLock) {
            localGamepad1 = g1;
            localGamepad2 = g2;
        }
        newDataReceived();
    }

    public void run() {
        while(true) {
            double left;
            double right;
            synchronized (gamepadLock)//get joystick values safely
            {
                left = localGamepad1.left_stick_y;
                right = localGamepad1.right_stick_y;
            }

            if(Math.abs(left) > .7 || Math.abs(right) > .7) {
                motorBL.setPower(left);
                motorFL.setPower(left);
                motorBR.setPower(right);
                motorFR.setPower(right);
            }

            newDataWait(5);
        }
    }
}

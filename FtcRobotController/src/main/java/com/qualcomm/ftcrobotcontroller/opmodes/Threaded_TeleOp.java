package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by FTC Robot Team 4211 on 8/11/2015.
 */
public class Threaded_TeleOp extends OpMode{

    Thread armThread;
    Thread driveThread;
    Arm_TeleOp armRunnable;
    Drive_TeleOp driveRunnable;

    @Override
    public void init() {
        armRunnable = new Arm_TeleOp(hardwareMap);
        armRunnable.passNewJoystickInfo(gamepad1, gamepad2);
        armThread = new Thread(armRunnable);

        driveRunnable = new Drive_TeleOp(hardwareMap);
        driveRunnable.passNewJoystickInfo(gamepad1, gamepad2);
        driveThread = new Thread(driveRunnable);
    }

    @Override
    public void start() {
        super.start();
        armThread.start(); // start Thread running here   if used in init() robot will move early/randomly
        driveThread.start();
    }

    @Override
    public void loop() {
        armRunnable.bufferedDeviceManager.swapInputBuffers();
        driveRunnable.bufferedDeviceManager.swapInputBuffers();
        armRunnable.passNewJoystickInfo(gamepad1, gamepad2);
        driveRunnable.passNewJoystickInfo(gamepad1, gamepad2);

        try
        {
            Thread.sleep(5); // pause to let loop run

            // if you don't take up more that 5 milliseconds there is a possibility that the Event Loop could call a wait
            // for an even longer period b/c it thinks there is a fast over processing condition
        }
        catch(Exception e)
        {}

        armRunnable.bufferedDeviceManager.swapOutputBuffers();
        driveRunnable.bufferedDeviceManager.swapOutputBuffers();
    }

    @Override
    public void stop() {
        super.stop();
        armThread.interrupt(); // make sure to stop the threads when the opMode ends
        driveThread.interrupt();
    }
}

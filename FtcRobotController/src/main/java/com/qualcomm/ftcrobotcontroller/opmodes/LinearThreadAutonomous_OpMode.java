package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by FTC Robot Team 4211 on 8/11/2015.
 */
public class LinearThreadAutonomous_OpMode extends OpMode{

    Thread armThread;
    Thread driveThread;
    AutoDrive_BufferedRunnable driveRunnable;

    @Override
    public void init() {

        driveRunnable = new AutoDrive_BufferedRunnable(hardwareMap);
        driveThread = new Thread(driveRunnable);
        armThread = new Thread(driveRunnable.armRunnable);//create thread for internal runnable
    }

    @Override
    public void start() {
        super.start();
        armThread.start(); // start Thread running here   if used in init() robot will move early/randomly
        driveThread.start();
    }

    @Override
    public void loop() {
        driveRunnable.armRunnable.bufferedDeviceManager.swapInputBuffers(); // swap from inside of other
        driveRunnable.bufferedDeviceManager.swapInputBuffers();

        try
        {
            Thread.sleep(5); // pause to let loop run

            // if you don't take up more that 5 milliseconds there is a possibility that the Event Loop could call a wait
            // for an even longer period b/c it thinks there is a fast over processing condition
        }
        catch(Exception e)
        {}

        driveRunnable.armRunnable.bufferedDeviceManager.swapOutputBuffers();
        driveRunnable.bufferedDeviceManager.swapOutputBuffers();
    }

    @Override
    public void stop() {
        super.stop();
        armThread.interrupt(); // make sure to stop the threads when the opMode ends
        driveThread.interrupt();
    }
}

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Bitmap;

import com.qualcomm.ftcrobotcontroller.NewRobotics;

/**
 * Created by viperbots on 9/29/2015.
 */
public class Autonomous_Testing extends LinearOpModeCamera {
    int ds2 = 2;  // additional downsampling of the image
    private int looped = 0;
    private long lastLoopTime = 0;
    // set to 1 to disable further downsampling

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    int count = 0;
    public void runOpMode() throws InterruptedException{
        if(isCameraAvailable()) {
            setCameraDownsampling(8);
            startCamera();
        }
        waitForStart();
        while(imageReady()) {
            Bitmap rgbImage;
            rgbImage = convertYuvImageToRgb(yuvImage, width, height, 2);
            telemetry.addData("dump? ", NewRobotics.should_climbers(rgbImage, telemetry));
            waitOneFullHardwareCycle();
        }
        sleep(300000000);
        super.stop();
    }
}

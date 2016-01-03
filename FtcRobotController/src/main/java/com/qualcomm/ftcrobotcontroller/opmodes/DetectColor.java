package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Bitmap;

import com.qualcomm.ftccommon.DbgLog;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class DetectColor extends OpModeCamera {

    int ds2 = 2;  // additional downsampling of the image
    private int looped = 0;
    private long lastLoopTime = 0;
    // set to 1 to disable further downsampling

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        setCameraDownsampling(8);
        // parameter determines how downsampled you want your images
        // 8, 4, 2, or 1.
        // higher number is more downsampled, so less resolution but faster
        // 1 is original resolution, which is detailed but slow
        // must be called before super.init sets up the camera

        super.init(); // inits camera functions, starts preview callback
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */

    @Override
    public void loop() {
        long startTime = System.currentTimeMillis();

        if (imageReady()) { // only do this if an image has been returned from the camera
            if(gamepad1.a)
            {
                Bitmap rgbImage;
                rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);
                String output = "";
                for (int x = 0; x < width / ds2; x++) {
                    for (int y = 0; y < height / ds2; y++) {
                        int pixel = rgbImage.getPixel(x, y);
                        output += color(pixel);
                    }
                }
                DbgLog.msg("My Name is Bo, Run #" + ++looped + ": " + output);
                int redValue = 0;
                int blueValue = 0;
                int greenValue = 0;

                for (int x = 0; x < width / ds2; x++) {
                    for (int y = 0; y < height / ds2; y++) {
                        int pixel = rgbImage.getPixel(x, y);
                        redValue += red(pixel);
                        blueValue += blue(pixel);
                        greenValue += green(pixel);
                    }
                }
                int color = highestColor(redValue, greenValue, blueValue);
                String colorString = "";
                switch (color) {
                    case 0:
                        colorString = "RED";
                        break;
                    case 1:
                        colorString = "GREEN";
                        break;
                    case 2:
                        colorString = "BLUE";
                }
                telemetry.addData("Color: ", colorString);
                telemetry.addData("My Name is Bo, Run #", looped +": " + output);
            }
        }
        long endTime = System.currentTimeMillis();
        telemetry.addData("Dims", Integer.toString(width / ds2) + " x " + Integer.toString(height / ds2));
        telemetry.addData("Loop Time", Long.toString(endTime - startTime));
        telemetry.addData("Loop to Loop Time", Long.toString(endTime - lastLoopTime));

        lastLoopTime = endTime;
    }

    @Override
    public void stop() {
        super.stop(); // stops camera functions
    }
    public String color(int pixel)
    {
        byte red = (byte)(pixel >> 16 & 0xFF);
        byte green = (byte)(pixel >> 8 & 0xFF);
        byte blue = (byte)(pixel & 0xFF);
        byte max = (byte) Math.max(red, Math.max(green,blue));
        if(red == max)
            return "R";
        if(blue == max)
            return "B";
        return "G";
    }
}

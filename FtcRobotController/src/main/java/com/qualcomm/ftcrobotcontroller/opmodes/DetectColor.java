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
    int count = 0;
    @Override
    public void loop() {
        long startTime = System.currentTimeMillis();

        if (imageReady() && count < 18) { // only do this if an image has been returned from the camera
            //if(gamepad1.a)
            //{
            count++;
            Bitmap rgbImage;
            rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);
            String output = "";
            /*DbgLog.msg(String.format("width = %d, length = %d", width / ds2, height / ds2));
            for (int x = 0, cnt = 0; x < width / ds2; x++) {
                for (int y = 0; y < height / ds2; y++, cnt++) {
                    int pixel = rgbImage.getPixel(x, y);
                    int red = (byte) (pixel >> 16 & 0xFF);
                    int green = (byte) (pixel >> 8 & 0xFF);
                    int blue = (byte) (pixel & 0xFF);
                    output += String.format("p%03d%03d%03d", red, green, blue);
                }
                output += "a";
            }
            DbgLog.msg("My Name is Bo, Run #" + String.format("%02d",count));

            for (int i = 0; i < output.length(); i += 3500) {
                DbgLog.msg("My Name is Bo, Run #" + String.format("%02d+%05d", count, i) + output.substring(i, i + 3500 < output.length() ? i + 3500 : output.length()));
            }*/
            String luminosity = "LUMINOSITY1: ";
            for (int x = 0, cnt = 0; x < width / ds2; x++) {
                for (int y = 0; y < height / ds2; y++, cnt++) {
                    int pixel = rgbImage.getPixel(x, y);
                    int red = (pixel >> 16 & 0xFF);
                    int green = (pixel >> 8 & 0xFF);
                    int blue = (pixel & 0xFF);
                    //output += String.format("%03d", (red + blue) / 2);
                    if(y > height / ds2 / 5 * 2 && y < height / ds2 / 5 * 3)
                        luminosity += (red + blue) / 2 + " ";
                }
                //output += "a";
                if(x > width / ds2 / 5 * 2 && x < width / ds2 / 5 * 3)
                    luminosity += "";
            }
            //for (int i = 0; i < output.length(); i += 3500) {
                //DbgLog.msg("My Name is Bo, Run #" + String.format("%02d+%05d", count, i) + output.substring(i, i + 3500 < output.length() ? i + 3500 : output.length()));
            //}
            DbgLog.msg(luminosity);
            telemetry.addData("count: ", count);

            //}
            //telemetry.addData("Color: ", colorString);
            //telemetry.addData("My Name is Bo, Run #", looped +": " + output);
        }
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

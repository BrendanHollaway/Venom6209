package com.qualcomm.ftcrobotcontroller.opmodes;

import android.hardware.Camera;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.io.ByteArrayOutputStream;
import java.util.Arrays;

import android.graphics.*;
import android.util.Log;
/**
 * Created by viperbots on 12/28/2015.
 */
public class CameraTest extends OpModeCamera
{
    public static final int width = 60;
    public static final int height = 80;
    public byte[][] RGB_photo = new byte[width][height]; // red = 0; blue = 1; everything else = -1;
    public int ds2 = 2;  // additional downsampling of the image
    private int looped = 0;
    private long lastLoopTime = 0;
    public Bitmap rgbImage;
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
    @Override
    public void loop() {
        long startTime = System.currentTimeMillis();

        if (imageReady()) { // only do this if an image has been returned from the camera
            int redValue = 0;
            int blueValue = 0;
            int greenValue = 0;
            rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);
            looped++;
            processPhoto();
            for(int b = 0; b < width / ds2; b++)
                DbgLog.msg(String.format("MY_NAME_IS_BO Picture Number: %d, line: %d: %s",looped, b, Arrays.toString(RGB_photo[b])));

        }
        long endTime = System.currentTimeMillis();
        telemetry.addData("Dims", Integer.toString(width / ds2) + " x " + Integer.toString(height / ds2));
        telemetry.addData("Loop Time", Long.toString(endTime - startTime));
        telemetry.addData("Loop to Loop Time", Long.toString(endTime - lastLoopTime));

        lastLoopTime = endTime;
    }
    public void processPhoto()
    {
        for (int x = 0; x < width / ds2; x++) {
            for (int y = 0; y < height / ds2; y++) {
                int pixel = rgbImage.getPixel(x, y);
                RGB_photo[x][y] = strongestColor(pixel);
            }
        }
    }
    public byte strongestColor(int pixel)
    {
        byte red = (byte)(pixel >> 16 & 0xff);
        byte green = (byte)(pixel >> 8 & 0xff);
        byte blue = (byte)(pixel & 0xff);
        byte max = (byte)(Math.max(red, Math.max(green, blue)));
        if(red == max)
            return 0;
        if(blue == max)
            return 1;
        return -1;
    }
}

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Bitmap;

import com.qualcomm.ftcrobotcontroller.opmodes.NewRobotics;
/**
 * Created by viperbots on 1/12/2016.
 */
public class cameraAutoTest extends LinearOpModeCamera{
    int ds2 = 2;

    @Override
    public void runOpMode()
    {
        setCameraDownsampling(8);
        while(true)
        {
            Bitmap rgbImage;
            rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);
            telemetry.addData("new Target: ", NewRobotics.fix_heading(rgbImage, ds2));
        }
    }

}

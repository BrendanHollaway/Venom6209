package com.qualcomm.ftcrobotcontroller.opmodes;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * Created by viperbots on 2/15/2016.
 */
public class BasicDumpAuto extends LinearOpModeCV {
    @Override
    public void runOpMode() throws InterruptedException {
        super.map();
        waitForVisionStart();
        //Set the camera used for detection
        this.setCamera(Cameras.PRIMARY);
        //Set the frame size
        //Larger = sometimes more accurate, but also much slower
        //For Testable OpModes, this might make the image appear small - it might be best not to use this
        this.setFrameSize(new Size(900, 900));

        //Enable extensions. Use what you need.
        enableExtension(VisionOpMode.Extensions.BEACON);     //Beacon detection
        enableExtension(VisionOpMode.Extensions.ROTATION);   //Automatic screen rotation correction

        //UNCOMMENT THIS IF you're using a SECONDARY (facing toward screen) camera
        //or when you rotate the phone, sometimes the colors swap
        //rotation.setRotationInversion(true);

        //You can do this for certain phones which switch red and blue
        //It will rotate the display and detection by 180 degrees, making it upright
        rotation.setUnbiasedOrientation(ScreenOrientation.PORTRAIT);

        //Set the beacon analysis method
        //Try them all and see what works!
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);

        //Wait for the match to begin
        waitForStart();
        motorFR.setPower(1);
        motorFL.setPower(1);
        motorBL.setPower(1);
        motorBR.setPower(1);
        wait(4000);
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(1);
        motorBL.setPower(1);
        wait(250);
        motorBL.setPower(0);
        motorFL.setPower(0);
        if (beacon.getAnalysis().getCenter().y / height > 0);

    }
}

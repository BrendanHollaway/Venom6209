package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.DPoint;
import com.qualcomm.ftcrobotcontroller.NewRobotics;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Mat;
import org.opencv.core.Size;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class BlueNoDumpAuto extends LinearOpModeCV {
    AutonomousSegments auto;

    @Override
    public void runOpMode() throws InterruptedException {
        //Wait for vision to initialize - this should be the first thing you do
        super.map();
        waitForVisionStart();
        DbgLog.error("Ready to start");
        auto = new AutonomousSegments(motorFL, motorBL, motorFR, motorBR, IMU, telemetry, this);
        //Set the camera used for detection
        this.setCamera(Cameras.PRIMARY);
        //Set the frame size
        //Larger = sometimes more accurate, but also much slower
        //For Testable OpModes, this might make the image appear small - it might be best not to use this
        this.setFrameSize(new Size(900, 900));

        //Enable extensions. Use what you need.
        enableExtension(Extensions.BEACON);     //Beacon detection
        enableExtension(Extensions.ROTATION);   //Automatic screen rotation correction

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
        resetStartTime();
        while(getRuntime() < 8)
            waitOneFullHardwareCycle();
        //Main loop
        //Camera frames and OpenCV analysis will be delivered to this method as quickly as possible
        //This loop will exit once the opmode is closed
        auto.Close_Blue_Buttons_CV_NoDump();
        while (opModeIsActive()) {
            //Log a few things
            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Location (Center)", beacon.getAnalysis().getLocationString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            telemetry.addData("New Heading", NewRobotics.calculate_heading(new DPoint(beacon.getAnalysis().getCenter().y, beacon.getAnalysis().getCenter().x)));
            telemetry.addData("Relative ", String.format("x: %.2f y: %.2f", beacon.getAnalysis().getCenter().y / height, beacon.getAnalysis().getCenter().x / width));
            //telemetry.addData("Rotation Compensation", rotation.getRotationCompensationAngle());
            //telemetry.addData("Frame Rate", fps.getFPSString() + " FPS");
            telemetry.addData("Frame Size", "Width: " + width + " Height: " + height); // width = 864, height = 480

            //You can access the most recent frame data and modify it here using getFrameRgba() or getFrameGray()
            //Vision will run asynchronously (parallel) to any user code so your programs won't hang
            //You can use hasNewFrame() to test whether vision processed a new frame
            //Once you copy the frame, discard it immediately with discardFrame()
            if (hasNewFrame()) {
                //Get the frame
                //Mat rgba = getFrameRgba();
                //Mat gray = getFrameGray();

                //Discard the current frame to allow for the next one to render
                discardFrame();
            }

            //Wait for a hardware cycle to allow other processes to run
            waitOneFullHardwareCycle();
        }
    }
}

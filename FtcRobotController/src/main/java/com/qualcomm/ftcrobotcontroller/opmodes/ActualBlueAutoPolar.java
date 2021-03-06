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
public class ActualBlueAutoPolar extends LinearOpModeCV {
    AutonomousSegments auto;
    @Override
    public void runOpMode() throws InterruptedException {
        //Wait for vision to initialize - this should be the first thing you do
        super.force_map();
        waitForVisionStart();
        auto = new AutonomousSegments(telemetry, this);
        //Set the camera used for detection
        this.setCamera(Cameras.PRIMARY);
        //Set the frame size
        //Larger = sometimes more accurate, but also much slower
        //For Testable OpModes, this might make the image appear small - it might be best not to use this
        this.setFrameSize(new Size(900, 900));

        /**
         * Enable extensions. Use what you need.
         * If you turn on the BEACON extension, it's best to turn on ROTATION too.
         */
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        /**
         * Set the beacon analysis method
         * Try them all and see what works!
         */
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         */
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        /**
         * Set the rotation parameters of the screen
         *
         * First, tell the extension whether you are using a secondary camera
         * (or in some devices, a front-facing camera that reverses some colors).
         *
         * For TestableVisionOpModes, changing other settings may break the app. See other examples
         * for normal OpModes.
         */
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE);
        //Set the beacon analysis method
        //Try them all and see what works!
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        IMU.offsetsInitialized = false;
        //Wait for the match to begin
        waitForStart();
        global_timeout = 29 * (long) Math.pow(10, 3) + System.currentTimeMillis(); // 29 milliseconds
        resetStartTime();
        /*while(getRuntime() < 8)
            waitOneFullHardwareCycle();*/
        auto.Close_Blue_Buttons_Polar();




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
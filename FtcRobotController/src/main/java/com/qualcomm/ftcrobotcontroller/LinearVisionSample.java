package com.qualcomm.ftcrobotcontroller;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.opmodes.AutonomousSegments;
import com.qualcomm.ftcrobotcontroller.opmodes.LinearOpModeCV2;
import com.qualcomm.robotcore.util.*;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.detection.objects.Rectangle;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.*;

/**
 * Linear Vision Sample
 * <p/>
 * Use this in a typical linear op mode. A LinearVisionOpMode allows using
 * Vision Extensions, which do a lot of processing for you. Just enable the extension
 * and set its options to your preference!
 * <p/>
 * Please note that the LinearVisionOpMode is specially designed to target a particular
 * version of the FTC Robot Controller app. Changes to the app may break the LinearVisionOpMode.
 * Should this happen, open up an issue on GitHub. :)
 */
public class LinearVisionSample extends LinearOpModeCV2 {

    //Frame counter
    int frameCount = 0;
    AutonomousSegments auto;

    @Override
    public void runOpMode() throws InterruptedException {
        //Wait for vision to initialize - this should be the first thing you do
        waitForVisionStart();
        super.force_map();
        //auto = new AutonomousSegments(telemetry, this);
        /**
         * Set the camera used for detection
         * PRIMARY = Front-facing, larger camera
         * SECONDARY = Screen-facing, "selfie" camera :D
         **/
        this.setCamera(Cameras.PRIMARY);

        /**
         * Set the frame size
         * Larger = sometimes more accurate, but also much slower
         * After this method runs, it will set the "width" and "height" of the frame
         **/
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
         * Set analysis boundary
         * You should comment this to use the entire screen and uncomment only if
         * you want faster analysis at the cost of not using the entire frame.
         * This is also particularly useful if you know approximately where the beacon is
         * as this will eliminate parts of the frame which may cause problems
         * This will not work on some methods, such as COMPLEX
         **/
        //beacon.setAnalysisBounds(new Rectangle(new Point(width / 2, height / 2), width - 200, 200));

        /**
         * Set the rotation parameters of the screen
         * If colors are being flipped or output appears consistently incorrect, try changing these.
         *
         * First, tell the extension whether you are using a secondary camera
         * (or in some devices, a front-facing camera that reverses some colors).
         *
         * It's a good idea to disable global auto rotate in Android settings. You can do this
         * by calling disableAutoRotate() or enableAutoRotate().
         *
         * It's also a good idea to force the phone into a specific orientation (or auto rotate) by
         * calling either setActivityOrientationAutoRotate() or setActivityOrientationFixed(). If
         * you don't, the camera reader may have problems reading the current orientation.
         */
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);
        beacon.setAnalysisBounds(new Rectangle(new org.opencv.core.Point(width / 2, height / 2), width, 300));

        /**
         * Set camera control extension preferences
         *
         * Enabling manual settings will improve analysis rate and may lead to better results under
         * tested conditions. If the environment changes, expect to change these values.
         */
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.K8000_SHADE);
        cameraControl.setAutoExposureCompensation();

        //Wait for the match to begin
        waitForStart();
        DbgLog.error("new heading: %.2f", find_Beacon());
        //Main loop
        //Camera frames and OpenCV analysis will be delivered to this method as quickly as possible
        //This loop will exit once the opmode is closed
        while (opModeIsActive()) {
            //Log a few things
            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Center", beacon.getAnalysis().getLocationString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            telemetry.addData("Beacon Buttons", beacon.getAnalysis().getButtonString());
            telemetry.addData("Screen Rotation", rotation.getScreenOrientationActual());
            telemetry.addData("Frame Rate", fps.getFPSString() + " FPS");
            telemetry.addData("Frame Size", "Width: " + width + " Height: " + height);
            telemetry.addData("Frame Counter", frameCount);

            //You can access the most recent frame data and modify it here using getFrameRgba() or getFrameGray()
            //Vision will run asynchronously (parallel) to any user code so your programs won't hang
            //You can use hasNewFrame() to test whether vision processed a new frame
            //Once you copy the frame, discard it immediately with discardFrame()
            if (hasNewFrame()) {
                //Get the frame
                Mat rgba = getFrameRgba();
                Mat gray = getFrameGray();

                //Discard the current frame to allow for the next one to render
                discardFrame();

                //Do all of your custom frame processing here
                //For this demo, let's just add to a frame counter
                frameCount++;
            }

            //Wait for a hardware cycle to allow other processes to run
            waitOneFullHardwareCycle();
        }
    }
    public double find_Beacon() throws InterruptedException
    {
        if(!opModeIsActive())
            return -1;
        resetStartTime();
        double total_heading = 0;
        double cnt = 0;
        double gyro_yaw;
        discardFrame();
        while(opModeIsActive() && getRuntime() < 5)
        {
            if(hasNewFrame())
            {
                gyro_yaw = 0;//getGyroYaw();
                DbgLog.error("finding beacon...");
                if(beacon.getAnalysis().getConfidence() > 0) { // beacon.getAnalysis().getConfidence() > 0.1 && <--- use this if COMPLEX analysis is on
                    if(!opModeIsActive())
                        return -1;
                    //tele.addData("beacon: ", String.format("change: %.2f", getHeading()));
                    DbgLog.error("Beacon Color" + beacon.getAnalysis().getColorString());
                    DbgLog.error("Beacon Location (Center)" + beacon.getAnalysis().getLocationString());
                    DbgLog.error("Beacon Confidence" + beacon.getAnalysis().getConfidenceString());
                    //DbgLog.error("New Heading: " + NewRobotics.calculate_heading(new DPoint(beacon.getAnalysis().getCenter().y, beacon.getAnalysis().getCenter().x)) + String.format(" old: %.2f", getGyroYaw()));
                    DbgLog.error("Relative " + String.format("y: %.2f x: %.2f", beacon.getAnalysis().getCenter().y / 480.0, beacon.getAnalysis().getCenter().x / 864.0));
                    //DbgLog.error("New Heading" + NewRobotics.calculate_heading(new DPoint(beacon.getAnalysis().getCenter().y, beacon.getAnalysis().getCenter().x)));
                    DbgLog.error(String.format("new heading target xy: %.2f, new yx: %.2f, old heading: %.2f", gyro_yaw +  getHeading(), 0.0, gyro_yaw));
                    if(!opModeIsActive())
                        return -1;
                    total_heading += getHeading();
                    if(!opModeIsActive())
                        return -1;
                    cnt++;
                    if(beacon.getAnalysis().isLeftKnown() && beacon.getAnalysis().isLeftRed())
                    {
                        //red_left_cnt++;
                    }
                    else if(beacon.getAnalysis().isLeftKnown() && beacon.getAnalysis().isLeftBlue())
                    {
                        //blue_left_cnt++;
                    }
                }
                waitOneFullHardwareCycle();
                //halt();
                discardFrame();
            }
            waitOneFullHardwareCycle();
        }
        DbgLog.error("cnt: " + cnt);
        if(cnt > 0)
            return (total_heading / cnt);
        else
            return 0;
    }
    public double getHeading() throws InterruptedException
    {
        if(!opModeIsActive())
            return -1;
        return this.calculate_heading(new DPoint(beacon.getAnalysis().getCenter().x, beacon.getAnalysis().getCenter().y), 0 / Math.pow(10, 3));
        //return NewRobotics.calculate_heading(DPoint.makeDPoint(beacon.getAnalysis().getCenter()), t / Math.pow(10, 6));
    }
    public double calculate_heading(DPoint center, double time) //encoder represents distance travelled. max_encoder is target distance
    {
        if(time > 3) // it is too close to the beacon by this point
            return 0;
        double ratio = 1.0 / 1.32; // width of view divided by distance to object- tested with ruler
        int width_pixels = 480; // width of screen
        double depth_inches = 18 * (1 - (time / 4)); //18
        double width_inches = depth_inches * ratio; // 13.64
        //double width_deg = Math.atan(ratio) * 2; // field of view of the camera, in degrees
        double center_x = center.x;
        double x_offset_pix = center_x - (width_pixels / 2); // in pixels
        double x_offset_inches = (x_offset_pix * width_inches / width_pixels); // phone is one inch from center
        double deg_offset = Math.toDegrees(Math.atan(x_offset_inches*2 / depth_inches)); // difference between current heading and target
        DbgLog.error(String.format("head: %.2f, x_off_in: %.2f, depth_in: %.2f, x_off_pix: %.2f", deg_offset, x_offset_inches, depth_inches, x_offset_pix));
        //tele.addData("head: ", String.format("%.2f, x_off_in: %.2f, depth_in: %.2f, x_off_pix: %.2f", deg_offset, x_offset_inches, depth_inches, x_offset_pix));
        return com.qualcomm.robotcore.util.Range.clip(deg_offset, -15, 15) / 1.8;
    }
}

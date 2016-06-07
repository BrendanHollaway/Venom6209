package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.DPoint;
import com.qualcomm.ftcrobotcontroller.NewRobotics;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.detection.objects.Rectangle;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Point;
import org.opencv.core.Size;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class WorldsRedAutoWait5 extends AutonomousSegments {
    @Override
    public void runOpMode() throws InterruptedException {
        //Wait for vision to initialize - this should be the first thing you do
        waitForVisionStart();
        super.force_map();
        /**
         * Set the camera used for detection
         * PRIMARY = Front-facing, larger camera
         * SECONDARY = Screen-facing, "selfie" camera :D
         **/
        this.setCamera(Cameras.PRIMARY);

        /**
         * Set the maximum frame size
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
         * Debug drawing
         * Enable this only if you're running test app - otherwise, you should turn it off
         * (Although it doesn't harm anything if you leave it on, only slows down image processing)
         */
        //beacon.enableDebug();

        /**
         * Set the rotation parameters of the screen
         *
         * First, tell the extension whether you are using a secondary camera
         * (or in some devices, a front-facing camera that reverses some colors).
         *
         * If you have a weird phone, you can set the "zero" orientation here as well.
         *
         * For TestableVisionOpModes, changing other settings may break the app. See other examples
         * for normal OpModes.
         */
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);
        beacon.setAnalysisBounds(new Rectangle(new Point(width / 2, height / 2), width, 300));

        /**
         * Set camera control extension preferences
         *
         * Enabling manual settings will improve analysis rate and may lead to better results under
         * tested conditions. If the environment changes, expect to change these values.
         */
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.K8000_SHADE);
        cameraControl.setAutoExposureCompensation();

        IMU.offsetsInitialized = false;
        //Wait for the match to begin
        waitForStart();
        global_timeout = 29 * (long) Math.pow(10, 3) + System.currentTimeMillis(); // 29 seconds
        resetStartTime();
        /*while(getRuntime() < 2) {
            waitOneFullHardwareCycle();
            motorPR.setPower(1);
            motorPL.setPower(1);
        }
        motorPR.setPower(0);
        motorPL.setPower(0);*/
        while(getRuntime() < 8)
            waitOneFullHardwareCycle();
        //motorM.setPower(1);

        Worlds_Align_Beacon_Red();
        Worlds_Set_Up_Red_Buttons();
        Worlds_Climbers();
        Worlds_Press_Red_Buttons();
        motorM.setPower(0);
        while (opModeIsActive()) {
            //Log a few things
            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Location (Center)", beacon.getAnalysis().getLocationString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            telemetry.addData("New Heading", NewRobotics.calculate_heading(new DPoint(beacon.getAnalysis().getCenter().x, beacon.getAnalysis().getCenter().y)));
            telemetry.addData("Relative ", String.format("x: %.2f y: %.2f", beacon.getAnalysis().getCenter().x / width, beacon.getAnalysis().getCenter().y / height));
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
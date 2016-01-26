package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Bitmap;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.opmodes.AutonomousSegments;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

public class blue_auto extends LinearOpMode2 {
    AutonomousSegments auto;

    public void runOpMode() throws InterruptedException {

        super.map();
        motorFL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        while(motorFL.getCurrentPosition() != 0 || motorFR.getCurrentPosition() != 0 || motorBR.getCurrentPosition() != 0 || motorBL.getCurrentPosition() != 0) {
            waitOneFullHardwareCycle();
        }
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        waitOneFullHardwareCycle();
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        auto = new AutonomousSegments(motorFL, motorBL, motorBR, motorFR, IMU, telemetry);
        /*long time = (long) (System.nanoTime() + Math.pow(10, 9.5));
        while(System.nanoTime() < time)
            telemetry.addData("gyro yaw; ", gyroTest());*/
        if(isCameraAvailable())
        {
            setCameraDownsampling(8);
            startCamera();
        }

        while (!opModeIsActive());
        if (opModeIsActive()) {


            waitForStart();
            telemetry.addData("autonomous: ", "Started");
            //long time = System.currentTimeMillis() + (int) Math.pow(10, 3);
            auto.Close_Red_Buttons();
            //double probability_score = 0;
            if(imageReady()) {
                Bitmap rgbImage;
                rgbImage = convertYuvImageToRgb(yuvImage, width, height, 2);
                if (NewRobotics.should_climbers(rgbImage, telemetry))
                    auto.Climbers();
            }
            //telemetry.addData("Probability = ", probability_score);
            //DbgLog.error("Probability = " + probability_score);
            //auto.Buttons();
            //auto.BlueButtons_RedRamp();
            motorFR.setDirection(DcMotor.Direction.FORWARD);
            motorBR.setDirection(DcMotor.Direction.FORWARD);
            telemetry.addData("Opmode Complete", ": nope");
            super.stop();
        }
        telemetry.addData("Opmode Complete", ": yep");
    }
    /*public double gyroTest() {
        IMU.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return yawAngle[0];
    }*/
}
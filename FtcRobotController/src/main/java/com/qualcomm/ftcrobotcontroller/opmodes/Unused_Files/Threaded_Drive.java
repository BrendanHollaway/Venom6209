package com.qualcomm.ftcrobotcontroller.opmodes.Unused_Files;

import com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs.BufferSupportedRunnable;
import com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs.BufferedInterpolatedMotor;
import com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs.BufferedMotor;
import com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs.BufferedUltrasonicSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by FTC Robot Team 4211 on 8/11/2015.
 */



/*
This program drives a robot forward while opening a claw, left while raising an arm, forward, closing a claw, backwards to a wall, and then lowers the arm

The AutoDriveRunnable is acting like the task main() in robotC
The arm runnable is owned here to allow the driveRunnable to pass information about what the armRunnable should be doing

//Note that this is simplified compared to what could be done, but provides a way that you can use it

Please don't run this sample as is, without first checking that it will work with your robot configuration
 */
public class Threaded_Drive extends BufferSupportedRunnable { //this

    BufferedInterpolatedMotor frontLeft;
    BufferedInterpolatedMotor frontRight;
    BufferedInterpolatedMotor backLeft;
    BufferedInterpolatedMotor backRight;

    BufferedUltrasonicSensor USSensor;
    public Threaded_Arm armRunnable; // own the runnable here so can pass info back and forth

    public Threaded_Drive(HardwareMap map) {
        super(map);

        USSensor = BufferedUltrasonicSensor.createBufferedUltrasonicSensor(map,"US Sensor");

        frontLeft = BufferedInterpolatedMotor.createBufferedInterpolatedMotor(map, "frontLeft");
        frontLeft.attachEncoder();
        frontLeft.hardwareResetBothAttachedEncodersValues();

        frontRight = BufferedInterpolatedMotor.createBufferedInterpolatedMotor(map, "frontRight");
        frontRight.attachEncoder();
        frontRight.hardwareResetBothAttachedEncodersValues();

        frontLeft.setBufferMode(BufferedMotor.BUFFER_METHOD.POWER_AND_ENCODER);
        frontLeft.resetEncoderValue();
        frontRight.setBufferMode(BufferedMotor.BUFFER_METHOD.POWER_AND_ENCODER);
        frontRight.resetEncoderValue();

        backLeft = BufferedInterpolatedMotor.createBufferedInterpolatedMotor(map, "Left");
        backLeft.attachEncoder();
        backLeft.hardwareResetBothAttachedEncodersValues();

        backRight = BufferedInterpolatedMotor.createBufferedInterpolatedMotor(map, "Right");
        backRight.attachEncoder();
        backRight.hardwareResetBothAttachedEncodersValues();

        backLeft.setBufferMode(BufferedMotor.BUFFER_METHOD.POWER_AND_ENCODER);
        backLeft.resetEncoderValue();
        backRight.setBufferMode(BufferedMotor.BUFFER_METHOD.POWER_AND_ENCODER);
        backRight.resetEncoderValue();

        armRunnable = new Threaded_Arm(map);
    }

    @Override
    public void newDataReceived() { // NOTE: Not called from this thread.  Called from opMode thread
        super.newDataReceived();
    }

    @Override
    public void run() { // this actually works with the new wrapper classes!!!

        //if(Math.abs())


       /* //drive forward
        driveLeft.setPower(1);
        driveRight.setPower(1);
        armRunnable.setClawControl(1); // open claw
        simpleWait(2500);

        // turn left
        driveLeft.setPower(-1);
        driveRight.setPower(1);
        armRunnable.setArmTarget(4000); // raise arm
        simpleWait(1300);

        //drive forward
        driveLeft.setPower(1);
        driveRight.setPower(1);
        simpleWait(2500);

        armRunnable.setClawControl(0); // close the claw
        simpleWait(750);

        //drive backward to 20cm from wall
        driveLeft.setPower(-1);
        driveRight.setPower(-1);
        while (USSensor.getBufferedDistanceReading()>20) // wait until sensor distance is < 20 cm
        {
            newDataWait(10); // wait until new data or for 10ms
        }


        //stop
        driveLeft.setPower(0);
        driveRight.setPower(0);
        armRunnable.setArmTarget(0); // reset arm
        simpleWait(60000);*/
    }
}

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs.BufferSupportedRunnable;
import com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs.BufferedMotor;
import com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs.BufferedServo;
import com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs.BufferedSmartMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by FTC Robot Team 4211 on 8/11/2015.
 */
public class AutoArm_BufferedRunnable extends BufferSupportedRunnable {

    BufferedSmartMotor armMotor;
    BufferedServo clawServo;
    Object controlLock = new Object();
    protected int target=0;
    protected double clawControl=0.5;

    public AutoArm_BufferedRunnable(HardwareMap map) {
        super(map);
        armMotor = BufferedSmartMotor.createBufferedSmartMotor(map,"Arm Motor"); // change names as necessary in ""
        clawServo = BufferedServo.createBufferedMotor(map, "Claw Servo");

        //setup motor for usage
        armMotor.attachEncoder();
        armMotor.setBufferMode(BufferedMotor.BUFFER_METHOD.POWER_AND_ENCODER);
        armMotor.enableSmartTargetSeeking(); // tell motor we want to use internal pid loop to seek
        armMotor.hardwareResetBothAttachedEncodersValues(); // call once per motor on a controller to reset
        armMotor.resetEncoderValue();
    }

    @Override
    public void newDataReceived() { // NOTE: Not called from this thread.  Called from opMode thread
        super.newDataReceived();
    }


    public int getArmTarget() {
        synchronized (controlLock) {
            return target;
        }
    }

    public void setArmTarget(int target) {
        synchronized (controlLock) {
            this.target = target;
        }
        newDataReceived();
    }

    public double getClawControl() {
        synchronized (controlLock) {
            return clawControl;
        }
    }

    public void setClawControl(double clawControl) {
        synchronized (controlLock) {
            this.clawControl = clawControl;
        }
        newDataReceived();
    }

    @Override
    public void run() {
        while (true) // perfectly valid to put infinite loop here or can run linear
        {

            clawServo.setPosition(clawControl);

            //logic for switching between manual control and PID position seeking provided by BufferedSmartMotor class
            armMotor.setAutoUpdatePower(true); // if false is passed in, PID loop runs but doesn't modify the motor power
            armMotor.setSmartTargetPosition(target); // allow PID to update power automatically

            armMotor.updatePIDLoop(); // always call last on smart motor

            newDataWait(5); // give rest time for something to change or for 5ms
                // this section immediately skips waiting in newDataAvailable() is called because of the notifyAll()
        }
    }
}

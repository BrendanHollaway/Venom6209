package com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorController;

public class BufferedMotor implements PhysicalDeviceBuffer {

    public enum BUFFER_METHOD {
        POWER,
        POWER_AND_ENCODER,
        POWER_ENCODER_AND_TARGET,
        FULL
    }

    protected DcMotor motor;
    public volatile String motorName;
    protected DcMotorController.RunMode bufferedRunMode = DcMotorController.RunMode.RUN_WITHOUT_ENCODERS;
    protected int bufferedTargetEncoderPosition = 0;
    protected double bufferedPower = 0;
    protected int lastEncoderPosition = 0;
    protected int encoderOffset = 0;
    protected BUFFER_METHOD bufferMode = BUFFER_METHOD.FULL;
    protected volatile boolean isEncoderAttached=false;
    protected volatile boolean isModeUpdate=false;
    protected Object bufferLock = new Object();


    public BufferedMotor(DcMotor underlyingMotor) {
        this(underlyingMotor, "Default Motor");
    }
    public BufferedMotor(DcMotor underlyingMotor, String cachedName) {
        motor = underlyingMotor;
        motorName = cachedName;
        this.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    public synchronized static BufferedMotor createBufferedMotor(HardwareMap map, String name) {
        return new BufferedMotor(map.dcMotor.get(name),name);
    }

    public void setDirection(DcMotor.Direction direction) {
        synchronized (bufferLock) {
            motor.setDirection(direction);
        }
    }

    public DcMotor.Direction getDirection() {
        synchronized (bufferLock) {
            return motor.getDirection();
        }
    }

    public BufferedMotor setBufferMode(BUFFER_METHOD method) {
        synchronized (bufferLock) {
            bufferMode = method;
            return this;
        }
    }

    public BUFFER_METHOD getBufferMode() {
        synchronized (bufferLock) {
            return bufferMode;
        }
    }

    public void setPower(double power) {
        synchronized (bufferLock) {
            bufferedPower = Math.max(Math.min(power,1),-1); // Range.clip(-1,1);
        }
    }

    public double getPower() {
        synchronized (bufferLock) {
            return bufferedPower;
        }
    }

    public void setTargetPosition(int position){
        synchronized (bufferLock) {
            bufferedTargetEncoderPosition = position;
        }
    }

    public int getTargetPosition() {
        synchronized (bufferLock) {
            return bufferedTargetEncoderPosition;
        }
    }

    public int getCurrentPosition() {
        synchronized (bufferLock) {
            return lastEncoderPosition-encoderOffset;
        }
    }

    public void attachEncoder() {
        synchronized (bufferLock) {
            isEncoderAttached=true;
            this.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
    }

    public void resetEncoderValue() {
        synchronized (bufferLock) {
            encoderOffset=lastEncoderPosition;
        }
    }

    public void hardwareResetBothAttachedEncodersValues() {
        this.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    public void setChannelMode(DcMotorController.RunMode mode) {
        synchronized (bufferLock) {
            bufferedRunMode = mode;
            isModeUpdate=true;
        }
    }

    public DcMotorController.RunMode getChannelMode() {
        synchronized (bufferLock) {
            return bufferedRunMode;
        }
    }


    @Override
    public void swapInputBuffers() {
        synchronized (bufferLock) {
            switch (bufferMode) {
                case POWER:
                    break;
                case POWER_AND_ENCODER:
                    lastEncoderPosition = motor.getCurrentPosition();
                    break;
                case POWER_ENCODER_AND_TARGET:
                    lastEncoderPosition = motor.getCurrentPosition();
                    break;
                case FULL:
                    lastEncoderPosition = motor.getCurrentPosition();
                    break;
            }
        }
    }

    @Override
    public void swapOutputBuffers() {
        synchronized (bufferLock) {
            if (isModeUpdate) {
                motor.setChannelMode(bufferedRunMode);
                if (isEncoderAttached)
                {
                    bufferedRunMode = DcMotorController.RunMode.RUN_USING_ENCODERS;
                }
                else
                {
                    bufferedRunMode = DcMotorController.RunMode.RUN_WITHOUT_ENCODERS;
                }
                isModeUpdate=false;
            }
            switch (bufferMode) {
                case POWER:
                    motor.setPower(bufferedPower);
                    break;
                case POWER_AND_ENCODER:
                    motor.setPower(bufferedPower);
                    break;
                case POWER_ENCODER_AND_TARGET:
                    motor.setPower(bufferedPower);
                    motor.setTargetPosition(bufferedTargetEncoderPosition);
                    break;
                case FULL:
                    motor.setPower(bufferedPower);
                    motor.setTargetPosition(bufferedTargetEncoderPosition);
                    break;
            }
        }
    }

    @Override
    public Object getUnsafeBase() {
        synchronized (bufferLock) {
            return motor;
        }
    }

    @Override
    public String toString()
    {
        return motorName+": "+bufferMode.toString();
    }
}

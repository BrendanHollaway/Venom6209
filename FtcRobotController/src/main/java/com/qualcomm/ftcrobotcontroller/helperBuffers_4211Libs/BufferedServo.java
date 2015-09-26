package com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BufferedServo implements PhysicalDeviceBuffer{

    protected Servo servo;
    public volatile String servoName;
    protected double bufferedPosition=0.5D;
    protected Object bufferLock = new Object();

    public BufferedServo(Servo underlyingServo) {
        this(underlyingServo,"Default Servo");
    }
    public BufferedServo(Servo underlyingServo, String cachedName) {
        servo=underlyingServo;
        servoName = cachedName;
    }

    public synchronized static BufferedServo createBufferedMotor(HardwareMap map, String name) {
        return new BufferedServo(map.servo.get(name), name);
    }

    public void setDirection(Servo.Direction direction)
    {
        synchronized (bufferLock) {
            servo.setDirection(direction);
        }
    }

    public Servo.Direction getDirection()
    {
        synchronized (bufferLock) {
            return servo.getDirection();
        }
    }

    public void setPosition(double position)
    {
        synchronized (bufferLock) {
            bufferedPosition = position;
        }
    }

    public double getPosition()
    {
        synchronized (bufferLock) {
            return bufferedPosition;
        }
    }

    public void scaleRange(double min, double max)
            throws IllegalArgumentException
    {
        synchronized (bufferLock) {
            servo.scaleRange(min,max);
        }
    }

    @Override
    public void swapInputBuffers() {
        synchronized (bufferLock)
        {
        }
    }

    @Override
    public void swapOutputBuffers() {
        synchronized (bufferLock)
        {
            servo.setPosition(bufferedPosition);
        }
    }

    @Override
    public Object getUnsafeBase() {
        synchronized (bufferLock) {
            return servo;
        }
    }

    public String toString()
    {
        return servoName;
    }
}

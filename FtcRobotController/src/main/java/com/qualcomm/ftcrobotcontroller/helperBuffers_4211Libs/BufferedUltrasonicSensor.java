package com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by FTC Robot Team 4211 on 7/31/2015.
 */
public class BufferedUltrasonicSensor implements PhysicalDeviceBuffer{
    protected UltrasonicSensor sensor;
    public volatile String sensorName;
    protected Object bufferLock = new Object();
    protected double bufferedUltrasonicReading =0;

    public BufferedUltrasonicSensor(UltrasonicSensor underlyingSensor)
    {
        this(underlyingSensor,"Default Light Sensor");
    }
    public BufferedUltrasonicSensor(UltrasonicSensor underlyingSensor, String cachedName)
    {
        sensor = underlyingSensor;
        sensorName = cachedName;
    }
    public synchronized static BufferedUltrasonicSensor createBufferedUltrasonicSensor(HardwareMap map, String name)
    {
        return new BufferedUltrasonicSensor(map.ultrasonicSensor.get(name),name);
    }

    public double getBufferedDistanceReading()
    {
        synchronized (bufferLock) {
            return bufferedUltrasonicReading;
        }
    }

    @Override
    public void swapInputBuffers() {
        synchronized (bufferLock)
        {
            bufferedUltrasonicReading = sensor.getUltrasonicLevel();
        }
    }

    @Override
    public void swapOutputBuffers() {
        synchronized (bufferLock)
        {
        }
    }

    @Override
    public Object getUnsafeBase() {
        synchronized (bufferLock) {
            return sensor;
        }
    }
}

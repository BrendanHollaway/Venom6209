package com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs;

import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by FTC Robot Team 4211 on 7/31/2015.
 */
public class BufferedGyroSensor implements PhysicalDeviceBuffer{
    protected GyroSensor sensor;
    public volatile String sensorName;
    protected Object bufferLock = new Object();
    protected double bufferedGyroReading =0;

    public BufferedGyroSensor(GyroSensor underlyingSensor)
    {
        this(underlyingSensor,"Default Light Sensor");
    }
    public BufferedGyroSensor(GyroSensor underlyingSensor, String cachedName)
    {
        sensor = underlyingSensor;
        sensorName = cachedName;
    }
    public synchronized static BufferedGyroSensor createBufferedGyroSensor(HardwareMap map, String name)
    {
        return new BufferedGyroSensor(map.gyroSensor.get(name),name);
    }

    public double getBufferedDistanceReading()
    {
        synchronized (bufferLock) {
            return bufferedGyroReading;
        }
    }

    @Override
    public void swapInputBuffers() {
        synchronized (bufferLock)
        {
            bufferedGyroReading = sensor.getRotation();
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

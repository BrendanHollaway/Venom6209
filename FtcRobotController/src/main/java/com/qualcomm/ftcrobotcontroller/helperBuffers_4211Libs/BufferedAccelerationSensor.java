package com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs;

import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by FTC Robot Team 4211 on 7/31/2015.
 */
public class BufferedAccelerationSensor implements PhysicalDeviceBuffer{
    protected AccelerationSensor sensor;
    public volatile String sensorName;
    protected Object bufferLock = new Object();
    protected AccelerationSensor.Acceleration bufferedAccelerationReading = new AccelerationSensor.Acceleration(0,0,0);

    public BufferedAccelerationSensor(AccelerationSensor underlyingSensor)
    {
        this(underlyingSensor,"Default Light Sensor");
    }
    public BufferedAccelerationSensor(AccelerationSensor underlyingSensor, String cachedName)
    {
        sensor = underlyingSensor;
        sensorName = cachedName;
    }
    public synchronized static BufferedAccelerationSensor createBufferedAccelerationSensor(HardwareMap map, String name)
    {
        return new BufferedAccelerationSensor(map.accelerationSensor.get(name),name);
    }

    public AccelerationSensor.Acceleration getBufferedAccelerationReading()
    {
        synchronized (bufferLock) {
            return bufferedAccelerationReading;
        }
    }

    @Override
    public void swapInputBuffers() {
        synchronized (bufferLock)
        {
            bufferedAccelerationReading = sensor.getAcceleration();
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

package com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;

/**
 * Created by FTC Robot Team 4211 on 7/31/2015.
 */
public class BufferedLightSensor implements PhysicalDeviceBuffer{
    protected LightSensor sensor;
    public volatile String sensorName;
    protected Object bufferLock = new Object();
    protected double bufferedLightReading=0;
    protected boolean ledStatus=false;

    public BufferedLightSensor(LightSensor underlyingSensor)
    {
        this(underlyingSensor,"Default Light Sensor");
    }
    public BufferedLightSensor(LightSensor underlyingSensor, String cachedName)
    {
        sensor = underlyingSensor;
        sensorName = cachedName;
    }
    public synchronized static BufferedLightSensor createBufferedLightSensor(HardwareMap map, String name)
    {
        return new BufferedLightSensor(map.lightSensor.get(name),name);
    }

    public double getBufferedLightReading()
    {
        synchronized (bufferLock) {
            return bufferedLightReading;
        }
    }

    public boolean getBufferedLedStatus()
    {
        synchronized (bufferLock) {
            return ledStatus;
        }
    }

    public void setBufferedLedStatus(boolean status)
    {
        synchronized (bufferLock) {
            ledStatus=status;
        }
    }

    @Override
    public void swapInputBuffers() {
        synchronized (bufferLock)
        {
            bufferedLightReading = sensor.getLightDetectedRaw();
        }
    }

    @Override
    public void swapOutputBuffers() {
        synchronized (bufferLock)
        {
            sensor.enableLed(ledStatus);
        }
    }

    @Override
    public Object getUnsafeBase() {
        synchronized (bufferLock) {
            return sensor;
        }
    }
}

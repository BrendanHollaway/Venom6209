package com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by FTC Robot Team 4211 on 8/4/2015.
 */
public class BufferedAnalogInput implements PhysicalDeviceBuffer{

    AnalogInput sensor;
    public volatile String sensorName;
    Object bufferLock = new Object();
    protected volatile int value=0;

    public BufferedAnalogInput(AnalogInput underlyingSensor)
    {
        this(underlyingSensor,"Default Analog Sensor");
    }
    public BufferedAnalogInput(AnalogInput underlyingSensor, String cachedName)
    {
        sensor = underlyingSensor;
        sensorName = cachedName;
    }
    public synchronized static BufferedAnalogInput createBufferedAnalogInput(HardwareMap map, String name)
    {
        return new BufferedAnalogInput(map.analogInput.get(name),name);
    }

    public int getValue()
    {
        synchronized (bufferLock)
        {
            return value;
        }
    }

    @Override
    public void swapInputBuffers() {
        synchronized (bufferLock)
        {
            value = sensor.getValue();
        }
    }

    @Override
    public void swapOutputBuffers() {

    }

    @Override
    public Object getUnsafeBase() {
        return sensor;
    }
}

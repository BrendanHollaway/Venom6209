package com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs;

import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by FTC Robot Team 4211 on 8/4/2015.
 */
public class BufferedAnalogOutput implements PhysicalDeviceBuffer{

    public enum AnalogOutputMode
    {
        DC,
        SINE_WAVE,
        SQUARE_WAVE,
        TRIANGLE;

        public static byte toByteRepresentation(AnalogOutputMode mode)
        {
            switch (mode)
            {
                case DC:
                    return 0;
                case SINE_WAVE:
                    return 1;
                case SQUARE_WAVE:
                    return 2;
                case TRIANGLE:
                    return 3;
                default:
                    return 0;
            }
        }
    }
    AnalogOutput sensor;
    public volatile String sensorName;
    Object bufferLock = new Object();
    protected volatile int bufferedFreq=0;
    protected volatile int bufferedVoltage=0;
    protected volatile AnalogOutputMode bufferedMode = AnalogOutputMode.DC;
    protected volatile boolean changeModes=false;
    protected volatile boolean changeValue=false;

    public BufferedAnalogOutput(AnalogOutput underlyingSensor)
    {
        this(underlyingSensor,"Default Analog Sensor");
    }
    public BufferedAnalogOutput(AnalogOutput underlyingSensor, String cachedName)
    {
        sensor = underlyingSensor;
        sensorName = cachedName;
        this.setOutputMethod(AnalogOutputMode.DC);
        this.setOutputFrequency(1000);
    }
    public synchronized static BufferedAnalogOutput createBufferedAnalogInput(HardwareMap map, String name)
    {
        return new BufferedAnalogOutput(map.analogOutput.get(name),name);
    }

    public void setOutputMethod(AnalogOutputMode mode)
    {
        synchronized (bufferLock)
        {
            bufferedMode = mode;
            changeModes=true;
        }
    }

    public void setOutputFrequency(int freq)
    {
        synchronized (bufferLock)
        {
            bufferedFreq=freq;
            changeModes=true;
        }
    }

    public void setOutputVoltage(int voltage)
    {
        synchronized (bufferLock)
        {
            bufferedVoltage=voltage;
            changeValue=true;
        }
    }

    @Override
    public void swapInputBuffers() {
    }

    @Override
    public void swapOutputBuffers() {
        synchronized (bufferLock)
        {
            if (changeModes)
            {
                sensor.setAnalogOutputMode(AnalogOutputMode.toByteRepresentation(bufferedMode));
                sensor.setAnalogOutputFrequency(bufferedFreq);
                changeModes=false;
            }
            if (changeValue)
            {
                sensor.setAnalogOutputVoltage(bufferedVoltage);
                changeValue=false;
            }
        }
    }

    @Override
    public Object getUnsafeBase() {
        return sensor;
    }
}

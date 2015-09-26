package com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PWMOutput;

/**
 * Created by FTC Robot Team 4211 on 8/4/2015.
 */
public class BufferedPWMOutput implements PhysicalDeviceBuffer{

    PWMOutput sensor;
    String sensorName;
    Object bufferLock = new Object();
    protected volatile int bufferedPeriod =0;
    protected volatile int bufferedTime =0;
    protected volatile boolean changeModes=false;
    protected volatile boolean changeValue=false;

    public BufferedPWMOutput(PWMOutput underlyingSensor)
    {
        this(underlyingSensor,"Default PWM Output");
    }
    public BufferedPWMOutput(PWMOutput underlyingSensor, String cachedName)
    {
        sensor = underlyingSensor;
        sensorName = cachedName;
        this.setOutputPeriod(1000);
    }
    public synchronized static BufferedPWMOutput createPWMOutput(HardwareMap map, String name)
    {
        return new BufferedPWMOutput(map.pwmOutput.get(name),name);
    }

    public void setOutputPeriod(int period)
    {
        synchronized (bufferLock)
        {
            bufferedPeriod =period;
            changeModes=true;
        }
    }

    public boolean setOutputTime(int time) // true if value acceptable
    {
        synchronized (bufferLock)
        {
            if (time>bufferedPeriod)
            {
                return false;
            }
            bufferedTime =time;
            changeValue=true;
            return true;
        }
    }

    public int getOutputTime() // true if value acceptable
    {
        synchronized (bufferLock)
        {
            return bufferedTime;
        }
    }

    public int getOutputPeriod() // true if value acceptable
    {
        synchronized (bufferLock)
        {
            return bufferedPeriod;
        }
    }

    public void setDutyCycle(double percentage, int period) // true if value acceptable
    {
        synchronized (bufferLock)
        {
            bufferedPeriod=period;
            bufferedTime=(int)(bufferedPeriod*percentage);
            changeValue=true;
            changeModes=true;
        }
    }

    public void setDutyCycle(double percentage) // true if value acceptable
    {
        synchronized (bufferLock)
        {
            setDutyCycle(percentage,bufferedPeriod);
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
                sensor.setPulseWidthPeriod(bufferedPeriod);
                changeModes=false;
            }
            if (changeValue)
            {
                sensor.setPulseWidthOutputTime(bufferedTime);
                changeValue=false;
            }
        }
    }

    @Override
    public Object getUnsafeBase() {
        return sensor;
    }
}

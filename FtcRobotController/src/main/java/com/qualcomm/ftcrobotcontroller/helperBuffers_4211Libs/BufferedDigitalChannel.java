package com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by FTC Robot Team 4211 on 8/4/2015.
 */
public class BufferedDigitalChannel implements PhysicalDeviceBuffer{

    DigitalChannel sensor;
    public volatile String sensorName;
    Object bufferLock = new Object();
    protected volatile boolean value=false;
    protected volatile DigitalChannelController.Mode inOutState= DigitalChannelController.Mode.INPUT;
    protected volatile boolean newState=false;

    public BufferedDigitalChannel(DigitalChannel underlyingSensor)
    {
        this(underlyingSensor,"Default Digital Sensor");
        sensor.setMode(DigitalChannelController.Mode.OUTPUT);
    }
    public BufferedDigitalChannel(DigitalChannel underlyingSensor, String cachedName)
    {
        sensor = underlyingSensor;
        sensorName = cachedName;
        this.setPinMode(DigitalChannelController.Mode.INPUT);
    }
    public synchronized static BufferedDigitalChannel createBufferedDigitalChannel(HardwareMap map, String name)
    {
        return new BufferedDigitalChannel(map.digitalChannel.get(name),name);
    }

    public void setPinMode(DigitalChannelController.Mode mode)
    {
        synchronized (bufferLock)
        {
            inOutState = mode;
            newState=true;
        }
    }

    public boolean getPinLogicValue()
    {
        synchronized (bufferLock)
        {
           return value;
        }
    }

    public int getPinIntValue()
    {
        synchronized (bufferLock)
        {
            if (value)
            {
                return 1;
            }
            return 0;
        }
    }

    public void setPinLogicValue(boolean state)
    {
        synchronized (bufferLock)
        {
            if (inOutState == DigitalChannelController.Mode.OUTPUT) {
                value = state;
            }
        }
    }

    public void setPinIntValue(int state)
    {
        synchronized (bufferLock)
        {
            if (inOutState == DigitalChannelController.Mode.OUTPUT) {
                if (state == 1) {
                    value = true;
                } else {
                    value = false;
                }
            }
        }
    }

    public void changeState()
    {
        synchronized (bufferLock)
        {
            if (inOutState == DigitalChannelController.Mode.OUTPUT) {
                value = !value;
            }
        }
    }

    @Override
    public void swapInputBuffers() {
        synchronized (bufferLock)
        {
            if(inOutState== DigitalChannelController.Mode.INPUT&&!newState) {
                value = sensor.getState();
            }
        }
    }

    @Override
    public void swapOutputBuffers() {
        synchronized (bufferLock) {
            if (newState) {
                sensor.setMode(inOutState);
                newState = false;
            }
            if (inOutState == DigitalChannelController.Mode.OUTPUT) {
                sensor.setState(value);
            }
        }

    }

    @Override
    public Object getUnsafeBase() {
        return sensor;
    }
}

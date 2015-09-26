package com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs;

import java.util.ArrayList;

/**
 * Created by FTC Robot Team 4211 on 7/31/2015.
 */
public class BufferedDeviceManager implements PhysicalDeviceBuffer{
    protected ArrayList<PhysicalDeviceBuffer> devices;
    protected ArrayList<physicalDataListener> listeners;
    protected Object bufferLock = new Object();

    public BufferedDeviceManager(){
        devices=new ArrayList<PhysicalDeviceBuffer>();
        listeners = new ArrayList<physicalDataListener>();
    }
    public void addDevice(PhysicalDeviceBuffer device)
    {
        synchronized (bufferLock) {
            devices.add(device);
        }
    }
    public void removeDevice(PhysicalDeviceBuffer device)
    {
        synchronized (bufferLock) {
            if (devices.contains(device)) {
                devices.remove(device);
            }
        }
    }
    public void addPhysicalDataListener(physicalDataListener listener)
    {
        synchronized (bufferLock) {
            listeners.add(listener);
        }
    }
    public void removePhysicalDataListener(physicalDataListener listener)
    {
        synchronized (bufferLock) {
            if (listeners.contains(listener)) {
                listeners.remove(listener);
            }
        }
    }

    @Override
    public void swapInputBuffers() {
        synchronized (bufferLock) {
            for (PhysicalDeviceBuffer b : devices) {
                b.swapInputBuffers();
            }
        }
        for(physicalDataListener d : listeners)
        {
            d.newDataReceived();
        }
    }

    @Override
    public void swapOutputBuffers() {
        synchronized (bufferLock) {
            for (PhysicalDeviceBuffer b : devices) {
                b.swapOutputBuffers();
            }
        }
    }

    @Override
    public Object getUnsafeBase() {
        return devices;
    }
}

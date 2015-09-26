package com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by FTC Robot Team 4211 on 7/31/2015.
 */
public abstract class BufferSupportedRunnable implements Runnable, physicalDataListener {
    public BufferedDeviceManager bufferedDeviceManager;
    public HardwareMap hMap;
    public BufferSupportedRunnable(HardwareMap map)
    {
        bufferedDeviceManager = new BufferedDeviceManager();
        bufferedDeviceManager.addPhysicalDataListener(this);
        hMap = map;
    }
    @Override
    public void newDataReceived() {
        this.notifyAll();
    }

    public void newDataWait(long waitTimeMs)
    {
        try
        {
            this.wait(waitTimeMs);
        }
        catch(Exception e)
        {
            System.out.println("The new data wait has failed... Crap");
        }
    }

    public void simpleWait(long waitTimeMs)
    {
        try
        {
            Thread.sleep(waitTimeMs);
        }
        catch(Exception e)
        {
            System.out.println("The simple wait has failed... Crap");
        }
    }

    @Override
    public void run() {

    }
}

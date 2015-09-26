package com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs;

/**
 * Created by FTC Robot Team 4211 on 7/31/2015.
 */
public interface PhysicalDeviceBuffer {
    public void swapInputBuffers();
    public void swapOutputBuffers();
    public Object getUnsafeBase();
}

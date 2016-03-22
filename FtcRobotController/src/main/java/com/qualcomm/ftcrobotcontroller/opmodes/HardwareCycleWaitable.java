package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Brendan Hollaway on 3/13/2016.
 */
public interface HardwareCycleWaitable {
    void waitOneFullHardwareCycle() throws InterruptedException;
}

package com.qualcomm.ftcrobotcontroller.opmodes.Unused_Files;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
/**
 * Created by viperbots on 11/14/2015.
 */
public class telemetryTest extends OpMode{
    public void init() {

    }
    public void loop() {
        telemetry.addData("pls", "woot");
    }
}

package com.qualcomm.ftcrobotcontroller.opmodes;


import com.lasarobotics.library.controller.ButtonState;
import com.lasarobotics.library.controller.Controller;
import com.lasarobotics.library.drive.Tank;
import com.lasarobotics.library.monkeyc.MonkeyC;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * MonkeyC2 Write Test
 */
public class MonkeyCWrite extends OpMode {
    //basic FTC classes
    DcMotor frontLeft, frontRight, backLeft, backRight;
    Controller one, two;

    MonkeyC writer;

    @Override
    public void init() {
        gamepad1.setJoystickDeadzone(.1F);
        gamepad2.setJoystickDeadzone(.1F);

        frontLeft = hardwareMap.dcMotor.get("lf");
        frontRight = hardwareMap.dcMotor.get("rf");
        backLeft = hardwareMap.dcMotor.get("lb");
        backRight = hardwareMap.dcMotor.get("rb");

        //frontLeft.setDirection(DcMotor.Direction.REVERSE);
        //frontRight.setDirection(DcMotor.Direction.REVERSE);

        one = new Controller(gamepad1);
        two = new Controller(gamepad2);
    }

    @Override
    public void start() {
        MonkeyCDo.isTested = false;
        writer = new MonkeyC();
    }

    @Override
    public void loop() {
        //update gamepads to controllers with events
        one.update(gamepad1);
        two.update(gamepad2);
        writer.add(one, two);

        if (one.x == ButtonState.PRESSED) {
            writer.pauseTime();
            MonkeyCDo.test();
            writer.waitForController(one, two);
        }

        if (MonkeyCDo.isTested) {
            telemetry.addData("X KEY", "PRESSED!");
        } else {
            telemetry.addData("X KEY", "Not pressed");
        }

        telemetry.addData("Status", writer.getCommandsWritten() + " commands written");
        telemetry.addData("Time", writer.getTime() + " seconds");

        //Drive commands go here (must match when playing back)
        Tank.motor4(frontLeft, frontRight, backLeft, backRight, -one.left_stick_y, one.right_stick_y);
    }

    @Override
    public void stop() {
        writer.write("test.txt", true);
    }
}
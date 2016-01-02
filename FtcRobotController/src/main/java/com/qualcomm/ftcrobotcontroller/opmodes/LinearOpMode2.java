//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public abstract class LinearOpMode2 extends LinearOpMode {
    private LinearOpMode2.a a = null;
    private Thread b = null;
    private ElapsedTime c = new ElapsedTime();
    private volatile boolean d = false;

    protected static DcMotor motorFR;
    protected static DcMotor motorFL;
    protected static DcMotor motorBR;
    protected static DcMotor motorBL;
    protected static DcMotor motorExtendLiftR;
    protected static DcMotor motorExtendLiftL;
    protected static DcMotor motorRaiseLiftR;
    protected static DcMotor motorRaiseLiftL;
    //protected Servo servoservoBucket;
    protected static Servo servoL;
    protected static Servo servoR;
    protected static Servo servoTopRatchet;
    protected Servo servoBotRatchet1;
    //protected Servo servoBotRatchet2;
    protected Servo servoClimberArm;
    protected AdafruitIMU IMU;
    protected volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    protected double[] accel = new double[3];
    static int encoderOffset = 0;

    protected void map() {
        motorBL = hardwareMap.dcMotor.get("bl");
        motorFL = hardwareMap.dcMotor.get("fl");
        motorBR = hardwareMap.dcMotor.get("br");
        motorFR = hardwareMap.dcMotor.get("fr");
        /*motorExtendLiftR = hardwareMap.dcMotor.get("ppr");
        motorExtendLiftL = hardwareMap.dcMotor.get("ppl");
        motorRaiseLiftR = hardwareMap.dcMotor.get("rlr");
        motorRaiseLiftL = hardwareMap.dcMotor.get("rll");
        servoL = hardwareMap.servo.get("climberArm"); //dont judge
        servoR = hardwareMap.servo.get("topRat");//dont judge
        servoClimberArm = hardwareMap.servo.get("lServo");
        servoTopRatchet = hardwareMap.servo.get("rServo");
        long systemTime = System.nanoTime();
        try {
            IMU = new AdafruitIMU(hardwareMap, "hydro"

                    //The following was required when the definition of the "I2cDevice" class was incomplete.
                    //, "cdim", 5

                    , (byte) (AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte) AdafruitIMU.OPERATION_MODE_IMU);
            telemetry.addData("IMU IS ALIVE: ", "NO ERRORS!");
        } catch (RobotCoreException e) {
            telemetry.addData("IMU IS DEAD: ", "IT THREW AN ERROR");
        }
        IMU.startIMU(); */
    }

    public LinearOpMode2() {
    }
    public abstract void runOpMode() throws InterruptedException;

    public synchronized void waitForStart() throws InterruptedException {
        while(!this.d) {
            synchronized(this) {
                this.wait();
            }
        }

    }

    public void waitOneFullHardwareCycle() throws InterruptedException {
        this.waitForNextHardwareCycle();
        Thread.sleep(1L);
        this.waitForNextHardwareCycle();
    }

    public void waitForNextHardwareCycle() throws InterruptedException {
        synchronized(this) {
            this.wait();
        }
    }

    public void sleep(long milliseconds) throws InterruptedException {
        Thread.sleep(milliseconds);
    }

    public boolean opModeIsActive() {
        return this.d;
    }


    private static class a implements Runnable {
        private RuntimeException a = null;
        private boolean b = false;
        private final LinearOpMode2 c;

        public a(LinearOpMode2 var1) {
            this.c = var1;
        }

        public void run() {
            this.a = null;
            this.b = false;

            try {
                this.c.runOpMode();
            } catch (InterruptedException var6) {
                RobotLog.d("LinearOpMode received an Interrupted Exception; shutting down this linear op mode");
            } catch (RuntimeException var7) {
                this.a = var7;
            } finally {
                this.b = true;
            }

        }

        public boolean a() {
            return this.a != null;
        }

        public RuntimeException b() {
            return this.a;
        }

        public boolean c() {
            return this.b;
        }

        public DcMotor getMotorFR()
        {
            return motorFR;
        }

        public Servo getServoZipLeft()
        {
            return servoL;
        }

        public void setEncoderFR(int encoder)
        {
            encoderOffset = motorFR.getCurrentPosition() - encoder;
        }

        public int getEncoderFR()
        {
            return motorFR.getCurrentPosition() - encoderOffset;
        }


    }
}


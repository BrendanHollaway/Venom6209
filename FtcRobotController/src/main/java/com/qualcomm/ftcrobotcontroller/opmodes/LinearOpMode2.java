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

    protected DcMotor motorFR;
    protected DcMotor motorFL;
    protected DcMotor motorBR;
    protected DcMotor motorBL;
    protected DcMotor motorPR;
    protected DcMotor motorPL;
    protected Servo servoL;
    protected Servo servoR;
    protected Servo servoClimberArm;
    protected Servo servoLRat;
    protected Servo servoRRat;
    protected AdafruitIMU IMU;
    protected volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    protected double[] accel = new double[3];
    int encoderOffset = 0;

    protected void map() {
        motorBL = hardwareMap.dcMotor.get("bl");
        motorFL = hardwareMap.dcMotor.get("fl");
        motorBR = hardwareMap.dcMotor.get("br");
        motorFR = hardwareMap.dcMotor.get("fr");
        motorPL = hardwareMap.dcMotor.get("pl");
        motorPR = hardwareMap.dcMotor.get("pr");
        servoLRat = hardwareMap.servo.get("lrat");
        servoRRat = hardwareMap.servo.get("rrat");
        servoClimberArm = hardwareMap.servo.get("arm");
        servoL = hardwareMap.servo.get("lservo");
        servoR = hardwareMap.servo.get("rservo");

        /*try {
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
    private class a implements Runnable {
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

        //public DcMotor getMotorFR() { return motorFR; }

        //public Servo getServoZipLeft() { return servoL; }

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


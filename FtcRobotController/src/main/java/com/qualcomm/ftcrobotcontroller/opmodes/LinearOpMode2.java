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

public abstract class LinearOpMode2 extends LinearOpModeCamera {
    private LinearOpMode2.a a = null;
    private Thread b = null;
    private ElapsedTime c = new ElapsedTime();
    private volatile boolean d = false;
    public boolean inited = false;
    public static final int ninety = 1/14;

    protected static DcMotor motorFR;
    protected static DcMotor motorFL;
    protected static DcMotor motorBR;
    protected static DcMotor motorBL;
    protected static DcMotor motorPR;
    protected static DcMotor motorPL;
    protected static Servo servoL;
    protected static Servo servoR;
    protected static Servo servoClimberArm;
    protected static Servo servoLRat;
    protected static Servo servoRRat;
    protected static Servo servoF;
    protected static Servo servoTL;
    protected static Servo servoTR;
    protected static Servo servoClimberHelper;
    protected static AdafruitIMU IMU;
    protected static volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    protected static double[] accel = new double[3];
    int encoderOffset = 0;

    protected void map() throws InterruptedException {
        motorBL = hardwareMap.dcMotor.get("bl");
        motorFL = hardwareMap.dcMotor.get("fl");
        motorBR = hardwareMap.dcMotor.get("br");
        motorFR = hardwareMap.dcMotor.get("fr");
        motorPL = hardwareMap.dcMotor.get("pl");
        motorPR = hardwareMap.dcMotor.get("pr");
        servoLRat = hardwareMap.servo.get("lrat");
        servoRRat = hardwareMap.servo.get("rrat");
        servoClimberArm = hardwareMap.servo.get("arm");
        servoL = hardwareMap.servo.get("lservo"); //yes this is correct
        servoR = hardwareMap.servo.get("rservo");
        servoF = hardwareMap.servo.get("servof");
        servoClimberHelper = hardwareMap.servo.get("servoH");
        //servoTL = hardwareMap.servo.get("servotl");
        //servoTR = hardwareMap.servo.get("servotr");

        if(IMU == null) {
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
        }
        else
        {
            telemetry.addData("IMU already init:", " true");
        }
        IMU.startIMU();
        servoRRat.setPosition(0.44);
        servoLRat.setPosition(0.5);
        servoClimberArm.setPosition(1);
        servoL.setPosition(1);
        servoR.setPosition(0.05);
        servoF.setPosition(0.5);
        servoClimberHelper.setPosition(1);
        motorPR.setDirection(DcMotor.Direction.REVERSE);
        //==========================RESET THE ENCODERS=========================
        /*motorFL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        while(motorFL.getCurrentPosition() != 0 || motorFR.getCurrentPosition() != 0 || motorBR.getCurrentPosition() != 0 || motorBL.getCurrentPosition() != 0) {
            waitOneFullHardwareCycle();
        }*/
        /*motorFL.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        /*motorFL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        waitOneFullHardwareCycle();
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);*/
        //======================END RESET THE ENCODERS=======================
        //if (IMU != null)
          //  inited = true;
        telemetry.addData("Init is Complete: ", "true");
        telemetry.addData("IMU is null: ", IMU == null);
    }

    public LinearOpMode2() {
    }
    public static boolean isInit()
    {
        return IMU != null && motorFL != null && motorFR != null && motorBL != null && motorBR != null;
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


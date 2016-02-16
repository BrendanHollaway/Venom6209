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
    public static final float ninety = 1/14.0f; // value required to move winch servo 90 degrees

    //Motors
    protected static DcMotor motorFR;
    protected static DcMotor motorFL;
    protected static DcMotor motorBR;
    protected static DcMotor motorBL;
    protected static DcMotor motorPR;
    protected static DcMotor motorPL;
    protected static DcMotor motorS; //front, the shield
    protected static DcMotor motorM; //manipulator


    //Servos
    protected static Servo servoL; //left zipliner
    protected static Servo servoR; //right zipliner
    protected static Servo servoClimberArm;
    protected static Servo servoRatL;
    protected static Servo servoRatR;
    //protected static Servo servoTread;
    //protected static Servo servoBasketL;
    //protected static Servo servoBasketR;
    protected static Servo servoAllClearL; // all clear left
    protected static Servo servoAllClearR; // all clear right

    //Other
    protected static AdafruitIMU IMU;
    protected static volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    protected static double[] accel = new double[3];
    int encoderOffset = 0;

    @Deprecated
    protected static Servo servoF;
    @Deprecated
    protected static Servo servoClimberHelper;

    protected void map() throws InterruptedException {
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorPL = hardwareMap.dcMotor.get("motorPR");
        motorPR = hardwareMap.dcMotor.get("motorPL");
        motorM  = hardwareMap.dcMotor.get("motorM");
        motorS  = hardwareMap.dcMotor.get("motorS");
        servoRatL = hardwareMap.servo.get("servoLRat");
        servoRatR = hardwareMap.servo.get("servoR");
        servoClimberArm = hardwareMap.servo.get("servoArm");
        servoL = hardwareMap.servo.get("servoL"); //yes this is correct
        servoR = hardwareMap.servo.get("servoRRat");
        //servoBasketL = hardwareMap.servo.get("servoBaskL");
        //servoBasketR = hardwareMap.servo.get("servoBaskR");
        //servoTread = hardwareMap.servo.get("servoTread");


        if(IMU == null) {
            try {
                IMU = new AdafruitIMU(hardwareMap, "IMU"

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
        //servoRRat.setPosition(0.44);
        //servoLRat.setPosition(0.5);
        //servoClimberArm.setPosition(1);
        servoL.setPosition(0);
        servoR.setPosition(.85);
        //servoBasketR.setPosition(.5);
        servoRatL.setPosition(.5);
        servoRatR.setPosition(.5);
        //servoBasketL.setPosition(.5);
        //servoF.setPosition(0.5);
        servoClimberArm.setPosition(0.06);
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


package com.qualcomm.ftcrobotcontroller.opmodes.Old_Autos;

import com.qualcomm.ftcrobotcontroller.opmodes.AutonomousSegments;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by viperbots on 10/12/2015.
 */
public class CBlue_BlueR extends LinearOpMode {
    DcMotor motorFL;
    DcMotor motorBL;
    DcMotor motorFR;
    DcMotor motorBR;
    AutonomousSegments auto;

    public void runOpMode() throws InterruptedException {

        motorFL = hardwareMap.dcMotor.get("motor_1");
        motorBL = hardwareMap.dcMotor.get("motor_2");
        motorFR = hardwareMap.dcMotor.get("motor_3");
        motorBR = hardwareMap.dcMotor.get("motor_4");
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //auto = new AutonomousSegments(motorFL, motorBL, motorBR, motorFR);

        auto.Close_Blue_BlueRamp();
        auto.ClearRamp();
        auto.ClimbRamp();
    }

    /**
     * Created by viperbots on 10/12/2015.
     */
    public static class CRed_B_BlueR extends LinearOpMode {
        DcMotor motorFL;
        DcMotor motorBL;
        DcMotor motorFR;
        DcMotor motorBR;
        AutonomousSegments auto;

        public void runOpMode() throws InterruptedException {

            motorFL = hardwareMap.dcMotor.get("motor_1");
            motorBL = hardwareMap.dcMotor.get("motor_2");
            motorFR = hardwareMap.dcMotor.get("motor_3");
            motorBR = hardwareMap.dcMotor.get("motor_4");
            motorFL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorBL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorFR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorBR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            //auto = new AutonomousSegments(motorFL, motorBL, motorBR, motorFR);

            auto.Close_Red_Buttons();
            auto.Climbers();
            auto.RedButtons_BlueRamp();
            auto.ClearRamp();
            auto.ClimbRamp();
        }
    }

    /**
     * Created by viperbots on 10/12/2015.
     */
    public static class CRed_B_RedR extends LinearOpMode {
        DcMotor motorFL;
        DcMotor motorBL;
        DcMotor motorFR;
        DcMotor motorBR;
        AutonomousSegments auto;

        public void runOpMode() throws InterruptedException {

            motorFL = hardwareMap.dcMotor.get("motor_1");
            motorBL = hardwareMap.dcMotor.get("motor_2");
            motorFR = hardwareMap.dcMotor.get("motor_3");
            motorBR = hardwareMap.dcMotor.get("motor_4");
            motorFL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorBL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorFR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorBR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            //auto = new AutonomousSegments(motorFL, motorBL, motorBR, motorFR);

            auto.Close_Red_Buttons();
            auto.Climbers();
            auto.RedButtons_RedRamp();
            auto.ClearRamp();
            auto.ClimbRamp();
        }
    }

    /**
     * Created by viperbots on 10/12/2015.
     */
    public static class CRed_BlueR extends LinearOpMode {
        DcMotor motorFL;
        DcMotor motorBL;
        DcMotor motorFR;
        DcMotor motorBR;
        AutonomousSegments auto;

        public void runOpMode() throws InterruptedException {

            motorFL = hardwareMap.dcMotor.get("motor_1");
            motorBL = hardwareMap.dcMotor.get("motor_2");
            motorFR = hardwareMap.dcMotor.get("motor_3");
            motorBR = hardwareMap.dcMotor.get("motor_4");
            motorFL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorBL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorFR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorBR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            //auto = new AutonomousSegments(motorFL, motorBL, motorBR, motorFR);

            auto.Close_Red_BlueRamp();
            auto.ClearRamp();
            auto.ClimbRamp();
        }
    }

    /**
     * Created by viperbots on 10/12/2015.
     */
    public static class CRed_RedR extends LinearOpMode {
        DcMotor motorFL;
        DcMotor motorBL;
        DcMotor motorFR;
        DcMotor motorBR;
        AutonomousSegments auto;

        public void runOpMode() throws InterruptedException {

            motorFL = hardwareMap.dcMotor.get("motor_1");
            motorBL = hardwareMap.dcMotor.get("motor_2");
            motorFR = hardwareMap.dcMotor.get("motor_3");
            motorBR = hardwareMap.dcMotor.get("motor_4");
            motorFL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorBL.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorFR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorBR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            //auto = new AutonomousSegments(motorFL, motorBL, motorBR, motorFR);

            auto.Close_Red_RedRamp();
            auto.ClearRamp();
            auto.ClimbRamp();
        }
    }
}


package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;

import com.qualcomm.ftcrobotcontroller.opmodes.AdafruitIMU;

/**
 * Created by Owner on 8/31/2015.
 */
public class IMUtest extends OpMode {

  AdafruitIMU boschBNO055;
  public double accel[] = new double[3];

  //The following arrays contain both the Euler angles reported by the IMU (indices = 0) AND the
  // Tait-Bryan angles ca+
  // lculated from the 4 components of the quaternion vector (indices = 1)
  volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];

  long systemTime;//Relevant values of System.nanoTime

  /************************************************************************************************
   * The following method was introduced in the 3 August 2015 FTC SDK beta release and it runs
   * before "start" runs.
   */
  @Override
  public void init() {
    telemetry.addData("Init: ", "Init begins");
    systemTime = System.nanoTime();
    try {
      boschBNO055 = new AdafruitIMU(hardwareMap, "hydro"

        //The following was required when the definition of the "I2cDevice" class was incomplete.
        //, "cdim", 5

      , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                                          //addressing
                                     , (byte)AdafruitIMU.OPERATION_MODE_IMU);
    } catch (RobotCoreException e){
      Log.i("FtcRobotController", "Exception: " + e.getMessage());
    }
    Log.i("FtcRobotController", "IMU Init method finished in: "
                                     + (-(systemTime - (systemTime = System.nanoTime()))) + " ns.");
    telemetry.addData("Init: ", "Init end");
    //ADDRESS_B is the "standard" I2C bus
    // address for the Bosch BNO055 (IMU data sheet, p. 90).
    //BUT DAVID PIERCE, MENTOR OF TEAM 8886, HAS EXAMINED THE SCHEMATIC FOR THE ADAFRUIT BOARD ON
    //WHICH THE IMU CHIP IS MOUNTED. SINCE THE SCHEMATIC SHOWS THAT THE COM3 PIN IS PULLED LOW,
    //ADDRESS_A IS THE IMU'S OPERATIVE I2C BUS ADDRESS
    //IMU is an appropriate operational mode for FTC competitions. (See the IMU datasheet, Table
    // 3-3, p.20 and Table 3-5, p.21.)
  }

  /************************************************************************************************
   * Code to run when the op mode is first enabled goes here
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
   */
  @Override
  public void start() {
        /*
      	* Use the hardwareMap to get the dc motors, servos and other sensors by name. Note
      	* that the names of the devices must match the names used when you
      	* configured your robot and created the configuration file. The hardware map
      	* for this OpMode is not initialized until the OpModeManager's "startActiveOpMode" method
      	* runs.
    		*/
    systemTime = System.nanoTime();
    boschBNO055.startIMU();//Set up the IMU as needed for a continual stream of I2C reads.
    Log.i("FtcRobotController", "IMU Start method finished in: "
                                    + (-(systemTime - (systemTime = System.nanoTime()))) + " ns.");
  }

  /***********************************************************************************************
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   * NOTE: BECAUSE THIS "loop" METHOD IS PART OF THE OVERALL OpMode/EventLoop/ReadWriteRunnable
   * MECHANISM, ALL THAT THIS METHOD WILL BE USED FOR, IN AUTONOMOUS MODE, IS TO:
   * 1. READ SENSORS AND ENCODERS AND STORE THEIR VALUES IN SHARED VARIABLES
   * 2. WRITE MOTOR POWER AND CONTROL VALUES STORED IN SHARED VARIABLES BY "WORKER" THREADS, AND
   * 3. SEND TELELMETRY DATA TO THE DRIVER STATION
   * THIS "loop" METHOD IS THE ONLY ONE THAT "TOUCHES" ANY SENSOR OR MOTOR HARDWARE.
   */
  @Override
  public void loop() {
    //Log.i("FtcRobotController", "Loop method starting at: " +
    //      -(systemTime - (systemTime = System.nanoTime())) + " since last loop start.");

    // write the values computed by the "worker" threads to the motors (if any)

    //Read the encoder values that the "worker" threads will use in their computations
    boschBNO055.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
    boschBNO055.getAccel(accel);
		/*
		 * Send whatever telemetry data you want back to driver station.
		 */
    //telemetry.addData("Text", "*** Robot Data***");
    telemetry.addData("Headings(yaw): ",
      String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));
    telemetry.addData("Pitches: ",
            String.format("Euler= %4.5f, Quaternion calculated= %4.5f", pitchAngle[0], pitchAngle[1]));
    telemetry.addData("Max I2C read interval: ",
      String.format("%4.4f ms. Average interval: %4.4f ms.", boschBNO055.maxReadInterval
        , boschBNO055.avgReadInterval));
    telemetry.addData("Accel ",
            String.format(" %4.5f,  %4.5f", accel[0], accel[1], accel[2]));
  }

  /*
  * Code to run when the op mode is first disabled goes here
  * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
  */
  @Override
  public void stop() {
    //When the FTC Driver Station's "Start with Timer" button commands autonomous mode to start,
    //then stop after 30 seconds, stop the motors immediately!
    //Following this method, the underlying FTC system will call
    // a "stop" routine of its own
    systemTime = System.nanoTime();
    Log.i("FtcRobotController", "IMU Stop method finished in: "
                                    + (-(systemTime - (systemTime = System.nanoTime()))) + " ns.");
  }
}

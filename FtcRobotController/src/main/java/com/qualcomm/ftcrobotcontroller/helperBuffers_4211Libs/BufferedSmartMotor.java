package com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by FTC Robot Team 4211 on 8/1/2015.
 */
public class BufferedSmartMotor extends BufferedInterpolatedMotor{

    protected double kP=0.002;
    protected double kI=0.008;
    protected double kD=0.01;
    protected double prevError =0;
    protected double iError=0;
    protected double iMax=1;
    protected double dError=0;
    protected int smartTargetPosition=0;
    protected int smartTargetDeadband=25;
    protected ElapsedTime smartTimer;
    protected boolean isSeeking=false;
    protected boolean isAtTarget=false;
    protected boolean autoUpdatePower=false;
    protected double smartMotorPower=0;

    public BufferedSmartMotor(DcMotor underlyingMotor) {
        super(underlyingMotor);
        smartTimer = new ElapsedTime();
    }

    public BufferedSmartMotor(DcMotor underlyingMotor, String cachedName) {
        super(underlyingMotor, cachedName);
        smartTimer = new ElapsedTime();
    }

    public synchronized static BufferedSmartMotor createBufferedSmartMotor(HardwareMap map, String name) {
        return new BufferedSmartMotor(map.dcMotor.get(name),name);
    }

    public double getD() {
        synchronized (bufferLock) {
            return kD;
        }
    }

    public void setD(double kD) {
        synchronized (bufferLock) {
            this.kD = kD;
        }
    }

    public void setPidCoefficients(double kP,double kI,double kD) {
        synchronized (bufferLock) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }

    public double getI() {
        synchronized (bufferLock) {
            return kI;
        }
    }

    public void setI(double kI) {
        synchronized (bufferLock) {
            this.kI = kI;
        }
    }

    public double getP() {
        synchronized (bufferLock) {
            return kP;
        }
    }

    public void setP(double kP) {
        synchronized (bufferLock) {
            this.kP = kP;
        }
    }

    public double getIMax() {
        synchronized (bufferLock) {
            return iMax;
        }
    }

    public void setIMax(double iMax) {
        synchronized (bufferLock) {
            this.iMax = iMax;
        }
    }

    public boolean isAtTarget() {
        synchronized (bufferLock) {
            return isAtTarget;
        }
    }

    public void enableSmartTargetSeeking() {
        synchronized (bufferLock) {
            isSeeking=true;
        }
    }

    public void disableSmartTargetSeeking() {
        synchronized (bufferLock) {
            isSeeking=false;
        }
    }

    public void setAutoUpdatePower(boolean autoUpdate) {
        synchronized (bufferLock) {
            autoUpdatePower=autoUpdate;
        }
    }

    public double getSmartMotorPower() {
        synchronized (bufferLock) {
            return smartMotorPower;
        }
    }

    public boolean getIsSmartTargetSeekingEnabled() {
        synchronized (bufferLock) {
            return isSeeking;
        }
    }

    public int getSmartTargetPosition() {
        synchronized (bufferLock) {
            return smartTargetPosition;
        }
    }

    public void setSmartTargetPosition(int target) {
        synchronized (bufferLock) {
            smartTargetPosition=target;
        }
    }

    public int getSmartTargetDeadband() {
        synchronized (bufferLock) {
            return smartTargetDeadband;
        }
    }

    public void setSmartTargetDeadband(int deadband) {
        synchronized (bufferLock) {
            smartTargetDeadband=deadband;
        }
    }

    public void resetInternalPIDLoop() {
        synchronized (bufferLock) {
            prevError =0;
            iError=0;
            dError=0;
        }
    }

    public void updatePIDLoop()
    {
        synchronized (super.bufferLock) {
            if (isSeeking&&smartTimer.time()>0.001) {
                double dt = smartTimer.time();
                int error = smartTargetPosition - getCurrentPosition();
                iError = iError + error*dt;
                if (iError!=0) {
                    iError = Math.abs(iError) > iMax ? iMax * Math.signum(iError) : iError; //iError / Math.abs(iError) : iError;
                }
                dError = (error - prevError)/dt;
                smartMotorPower = Math.max(Math.min(kP*error + kI*iError + kD*dError,1),-1); //obtains power of the motor, based on PID
                prevError = error;
                if (Math.abs(error)<smartTargetDeadband)
                {
                    isAtTarget=true;
                }
                else
                {
                    isAtTarget=false;
                }
                if (autoUpdatePower)
                {
                    setPower(smartMotorPower);
                }
            }
        }
    }
}

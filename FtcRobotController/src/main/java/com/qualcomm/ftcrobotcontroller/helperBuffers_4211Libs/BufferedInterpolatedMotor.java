package com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by FTC Robot Team 4211 on 7/31/2015.
 */
public class BufferedInterpolatedMotor extends BufferedMotor{

    protected ElapsedTime timer;
    private double[] prevDtArray;
    private double[] prevErrorArray;
    private int dtIndex=0;
    protected double aveDtInterval=0;
    protected double aveValueError=0;
    protected double encoderTicksPerDt=0;
    protected double encoderTicksPerDtPerDt=0;

    public BufferedInterpolatedMotor(DcMotor underlyingMotor, String cachedName) {
        super(underlyingMotor, cachedName);
        timer = new ElapsedTime();
        prevDtArray = new double[5];
        prevErrorArray = new double[5];
    }

    public BufferedInterpolatedMotor(DcMotor underlyingMotor) {
        super(underlyingMotor);
        timer = new ElapsedTime();
        prevDtArray=new double[5];
        prevErrorArray=new double[5];
    }

    public synchronized static BufferedInterpolatedMotor createBufferedInterpolatedMotor(HardwareMap map, String name) {
        return new BufferedInterpolatedMotor(map.dcMotor.get(name),name);
    }


    @Override
    public void setPower(double power) {
        synchronized (super.bufferLock) {
            if (bufferedPower!=0) {
                encoderTicksPerDt *= power / bufferedPower;
                encoderTicksPerDtPerDt *= power / bufferedPower;
            }
            else
            {
                encoderTicksPerDt = bufferedPower/2500;
                encoderTicksPerDtPerDt = bufferedPower/2500;
            }
            super.setPower(power);
        }
    }

    public double getAverageDtInterval()
    {
        synchronized (super.bufferLock)
        {
            return aveDtInterval;
        }
    }

    public double getAveragePredictedValueError() {
        synchronized (super.bufferLock) {
            return aveValueError;
        }
    }

    public double getEncoderTicksPerDt() {
        synchronized (super.bufferLock) {
            return encoderTicksPerDt;
        }
    }

    public int getPredictedEncoderValue()
    {
        synchronized (super.bufferLock)
        {
            double workTime = timer.time();
            return (int)(super.lastEncoderPosition + encoderTicksPerDt*workTime + encoderTicksPerDtPerDt*workTime*workTime -aveValueError);
        }
    }
    public int getPredictedEncoderValue(double futureSecondsFromNow)
    {
        synchronized (super.bufferLock)
        {
            double workTime = timer.time();
            return (int)(super.lastEncoderPosition + encoderTicksPerDt*(workTime+futureSecondsFromNow) + encoderTicksPerDtPerDt*(workTime+futureSecondsFromNow)*(workTime+futureSecondsFromNow) - aveValueError);
        }
    }

    public double getPredictedPowerToReachPositionDtFromNow(int targetPosition, double futureSecondsFromNow)
    {
        synchronized (super.bufferLock)
        {
            if (bufferedPower!=0&&encoderTicksPerDt!=0&&futureSecondsFromNow!=0) {
                double ticksPerDtToTarget = (targetPosition - lastEncoderPosition) / futureSecondsFromNow;
                double ratio = ticksPerDtToTarget / encoderTicksPerDt;
                return bufferedPower * ratio;
            }
            return Math.max(Math.min(targetPosition-lastEncoderPosition,1),-1);
        }
    }
    public double getPredictedEta(int targetPosition)
    {
        synchronized (super.bufferLock)
        {
            if (encoderTicksPerDt!=0) {
                return (targetPosition-getPredictedEncoderValue())/encoderTicksPerDt;
            }
            return -1;
        }
    }

    @Override
    public void swapInputBuffers(){
        synchronized (super.bufferLock) {
            int cachedEncoderValue=super.lastEncoderPosition;
            super.swapInputBuffers();
            prevDtArray[dtIndex] = timer.time();
            prevErrorArray[dtIndex]=getPredictedEncoderValue()-super.lastEncoderPosition;
            timer.reset();
            double temp=encoderTicksPerDt;
            encoderTicksPerDt = (super.lastEncoderPosition-cachedEncoderValue)/prevDtArray[dtIndex];
            encoderTicksPerDtPerDt = (encoderTicksPerDt-temp)/prevDtArray[dtIndex];
            dtIndex = (dtIndex+1)%prevDtArray.length;
            aveDtInterval=0;
            aveValueError=0;
            for (int i=0; i<prevDtArray.length; i++)
            {
                aveDtInterval+=prevDtArray[i];
                aveValueError+=prevErrorArray[i];
            }
            aveDtInterval = aveDtInterval/prevDtArray.length;
            aveValueError = aveValueError/prevErrorArray.length;
        }
    }
}

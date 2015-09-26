package com.qualcomm.ftcrobotcontroller.helperBuffers_4211Libs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;

import java.util.ArrayList;

/**
 * Created by FTC Robot Team 4211 on 8/4/2015.
 */
public class BufferedI2CSensor implements PhysicalDeviceBuffer{


    public enum I2CPolicy
    {
        TRANSFER_ALL,
        TRANSFER_ONE,
        TRANSFER_TWO,
        TRANSFER_NONE;
        public static byte toByteRepresentation(I2CPolicy mode)
        {
            switch (mode)
            {
                case TRANSFER_ALL:
                    return Byte.MAX_VALUE;
                case TRANSFER_ONE:
                    return 1;
                case TRANSFER_TWO:
                    return 2;
                case TRANSFER_NONE:
                    return 0;
                default:
                    return 1;
            }
        }
    }
    I2cDevice sensor;
    public volatile String sensorName;
    Object bufferLock = new Object();
    protected ArrayList<byte[]> i2cOut;
    protected ArrayList<byte[]> i2cReceived;
    protected I2CPolicy i2cWriteReadPolicy;
    protected int expectedI2CBack =0;

    public BufferedI2CSensor(I2cDevice underlyingSensor)
    {
        this(underlyingSensor,"Default I2C Sensor");
    }
    public BufferedI2CSensor(I2cDevice underlyingSensor, String cachedName)
    {
        sensor = underlyingSensor;
        sensorName = cachedName;
        i2cOut = new ArrayList<byte[]>();
        i2cReceived = new ArrayList<byte[]>();
        i2cWriteReadPolicy=I2CPolicy.TRANSFER_NONE;
    }
    public synchronized static BufferedI2CSensor createBufferedI2CSensor(HardwareMap map, String name)
    {
        return new BufferedI2CSensor(map.i2cDevice.get(name),name);
    }

    public void queueI2CWriteCommand(byte i2cDeviceAddress, byte memAddress, byte[] data)
    {
        byte[] temp = new byte[data.length+3];
        temp[0]=0;
        temp[1]=i2cDeviceAddress;
        temp[2]=memAddress;
        for (int i=0; i<data.length; i++)
        {
            temp[i+3]=data[i];
        }
        synchronized (bufferLock) {
            i2cOut.add(temp);
        }
    }
    public void queueI2CReadCommand(byte i2cDeviceAddress, byte memAddress, byte dataLengthBack)
    {
        byte[] temp = new byte[4];
        temp[0]=Byte.MIN_VALUE;
        temp[1]=i2cDeviceAddress;
        temp[2]=memAddress;
        temp[3]=dataLengthBack;
        synchronized (bufferLock) {
            i2cOut.add(temp);
            expectedI2CBack++;
        }
    }

    public void queueI2CRawCommand(byte[] data, boolean isReadCommand)
    {
        synchronized (bufferLock) {
            if (isReadCommand)
            {
                expectedI2CBack++;
            }
            i2cOut.add(data);
        }
    }

    public void setI2cReadWritePolicy(I2CPolicy policy)
    {
        synchronized (bufferLock) {
            i2cWriteReadPolicy=policy;
        }
    }

    public I2CPolicy getI2cReadWritePolicy()
    {
        synchronized (bufferLock) {
            return i2cWriteReadPolicy;
        }
    }

    public void clearI2CWriteAndReadQueue()
    {
        synchronized (bufferLock) {
            i2cOut.clear();
        }
    }

    public void clearI2CReceivedDataQueue()
    {
        synchronized (bufferLock) {
            i2cReceived.clear();
        }
    }

    public byte[] popFirstReceivedData()
    {
        synchronized (bufferLock) {
            return i2cReceived.remove(0);
        }
    }

    public byte[] peekFirstReceivedData()
    {
        synchronized (bufferLock) {
            return i2cReceived.get(0);
        }
    }

    public byte[] peekLastReceivedData()
    {
        synchronized (bufferLock) {
            return i2cReceived.get(i2cReceived.size()-1);
        }
    }

    public byte[] popLastReceivedData()
    {
        synchronized (bufferLock) {
            return i2cReceived.remove(i2cReceived.size() - 1);
        }
    }

    public byte[] getI2CRawReadCacheCopy()
    {
        synchronized (bufferLock) {
            byte[] temp = new byte[6];
            try {
                sensor.getI2cReadCacheLock().lock();
                temp=sensor.getI2cReadCache().clone();
            }
            finally {
                sensor.getI2cReadCacheLock().unlock();
            }
            return temp;
        }
    }
    public byte[] getI2CRawWriteCacheCopy()
    {
        synchronized (bufferLock) {
            byte[] temp = new byte[6];
            try {
                sensor.getI2cWriteCacheLock().lock();
                temp=sensor.getI2cWriteCache().clone();
            }
            finally {
                sensor.getI2cWriteCacheLock().unlock();
            }
            return temp;
        }
    }

    public int getI2CReadCacheSize()
    {
        synchronized (bufferLock) {
            int temp = -1;
            try {
                sensor.getI2cReadCacheLock().lock();
                temp=sensor.getI2cReadCache().length;
            }
            finally {
                sensor.getI2cReadCacheLock().unlock();
            }
            return temp;
        }
    }
    public int getI2CWriteCacheSize()
    {
        synchronized (bufferLock) {
            int temp = -1;
            try {
                sensor.getI2cWriteCacheLock().lock();
                temp=sensor.getI2cWriteCache().length;
            }
            finally {
                sensor.getI2cWriteCacheLock().unlock();
            }
            return temp;
        }
    }

    protected void readI2CData()
    {
        synchronized (bufferLock) {
            int times=0;
            if (expectedI2CBack>0&&i2cWriteReadPolicy!=I2CPolicy.TRANSFER_NONE)
            {
                byte[] data = new byte[2];
                do {
                    try {
                        sensor.getI2cReadCacheLock().lock();
                        data = sensor.getI2cReadCache();
                        if (data!=i2cReceived.get(i2cReceived.size()-1))
                        {
                            i2cReceived.add(data);
                            expectedI2CBack--;
                        }
                    } finally {
                        sensor.getI2cReadCacheLock().unlock();
                    }
                    times++;
                }while (data!=i2cReceived.get(i2cReceived.size()-1)&&times<I2CPolicy.toByteRepresentation(i2cWriteReadPolicy));
            }
        }
    }

    protected void writeI2CData()
    {
        synchronized (bufferLock) {
            byte numberToRead = I2CPolicy.toByteRepresentation(i2cWriteReadPolicy);
            byte countMore=0;
            while(i2cOut.size()>0&&countMore<numberToRead)
            {
                try
                {
                    countMore++;
                    sensor.getI2cWriteCacheLock().lock();
                    byte[] cache = sensor.getI2cWriteCache();
                    byte[] tempCmd = i2cOut.remove(0);
                    System.arraycopy(tempCmd, 0, cache, 0, Math.min(cache.length,tempCmd.length));
                }
                finally {
                    sensor.getI2cWriteCacheLock().unlock();
                }

            }
        }
    }



    @Override
    public void swapInputBuffers() {
        synchronized (bufferLock)
        {
            readI2CData();
        }
    }

    @Override
    public void swapOutputBuffers() {
        synchronized (bufferLock)
        {
            writeI2CData();
        }
    }

    @Override
    public Object getUnsafeBase() {
        return sensor;
    }
}

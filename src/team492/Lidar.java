package team492;

import java.nio.ByteBuffer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import trclib.TrcDbgTrace;

public class Lidar
{
    private static final String moduleName = "Lidar";
    private static final boolean debugEnabled = true;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = true;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private I2C i2c;
    private java.util.Timer updater;
    private ByteBuffer buffer = ByteBuffer.allocateDirect(3);
    private volatile int distance;
    private int measurementCount = 0;

    private static final int LIDAR_BUSY_MASK = 0x01;
    private static final int LIDAR_COMMAND_ACQUIRE_WITHOUT_CORRECTION = 0x03;
    private static final int LIDAR_COMMAND_ACQUIRE_WITH_CORRECTION = 0x04;
    private static final int LIDAR_CONFIG_REGISTER = 0x00;
    private static final int LIDAR_STATUS_REGISTER = 0x01;
    private static final int LIDAR_SIG_COUNT = 0x02;
    private static final int LIDAR_ACQ_CONFIG = 0x04;
    private static final int LIDAR_THRESHOLD_BYPASS = 0x1c;
    private static final int LIDAR_DISTANCE_REGISTER = 0x8f;

    private static final int UPDATE_PERIOD = 20; // in milliseconds
    private static final int RETRY_COUNT = 500;

    public Lidar(String instanceName, Port port, byte address)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        i2c = new I2C(port, address);

        setup();

        updater = new java.util.Timer();
        updater.schedule(new TimerTask()
        {
            @Override
            public void run()
            {
                distance = getUpdatedDistance();
            }
        }, 0, UPDATE_PERIOD);
        System.out.println("Started");
    }

    // Distance in cm
    public int getDistance()
    {
        return distance;
    }

    public void setup()
    {
        final String funcName = "setup";
        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "writing 0x80 to SIG_COUNT");
        }
        i2c.write(LIDAR_SIG_COUNT, 0x80);
        sleep(1);

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "writing 0x08 to ACQ_CONFIG");
        }
        i2c.write(LIDAR_ACQ_CONFIG, 0x08);
        sleep(1);

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "writing 0x00 to THESHOLD_BYPASS");
        }
        i2c.write(LIDAR_THRESHOLD_BYPASS, 0x00);
        sleep(1);
    }

    // Update distance variable
    private int getUpdatedDistance()
    {
        final String funcName = "getUpdatedDistance";
//        int command = (measurementCount % 100 == 0 ? LIDAR_COMMAND_ACQUIRE_WITH_CORRECTION
//            : LIDAR_COMMAND_ACQUIRE_WITHOUT_CORRECTION);
        int command = LIDAR_COMMAND_ACQUIRE_WITH_CORRECTION;

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "writing 0x%02x to COMMAND, count=%d", command, measurementCount);
        }
        i2c.write(LIDAR_CONFIG_REGISTER, command); // Initiate measurement
        /*
         * if (measurementCount++ % 50 == 0) { System.out.println("count = " +
         * measurementCount + ", distance = " + distanceValue); }
         */
        measurementCount++;
        int busyCount = 0;
        do
        {
            sleep(1);
            int status = readByte(LIDAR_STATUS_REGISTER);

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "[%d] status=0x%02x", busyCount, status);
            }
            boolean busy = (status & LIDAR_BUSY_MASK) == LIDAR_BUSY_MASK;
            if (!busy)
            {
                int cmDistance = readShort(LIDAR_DISTANCE_REGISTER);
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "[busyCount=%d] distance=0x%04x cm", busyCount, cmDistance);
                }
                return cmDistance;
            }
            else
            {
                busyCount++;
            }
            /*
             * SmartDashboard.putNumber("status", status);
             * SmartDashboard.putBoolean("busyFlag", busy);
             */
        } while (busyCount < RETRY_COUNT);
        System.out.println("Distance read timed out");
        return distance;
    }

    private void sleep(long millis)
    {
        try
        {
            Thread.sleep(millis);
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }
    }

    private int readByte(int register)
    {
        buffer.put(0, (byte) register);
        i2c.writeBulk(buffer, 1);
        i2c.readOnly(buffer, 1);
        return buffer.get(0) & 0xFF;
    }

    private int readShort(int register)
    {
        buffer.put(0, (byte) register);
        i2c.writeBulk(buffer, 1);
        i2c.readOnly(buffer, 2);
        return buffer.getShort(0) & 0xFFFF;
    }

}

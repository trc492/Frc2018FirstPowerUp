/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frclib;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import trclib.TrcDbgTrace;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

/**
 * This class extends the WPI PowerDistricbutonPanel class to provide a monitor on the power consumption of each
 * power channel.
 */
public class FrcPdp extends PowerDistributionPanel
{
    private static final String moduleName = "FrcPdp";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public static final int NUM_PDP_CHANNELS = 16;
    
    private final TrcTaskMgr.TaskObject pdpEnergyTaskObj;
    private double[] energyUsed = new double[NUM_PDP_CHANNELS];
    private double lastTimestamp = 0.0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param canId specifies the CAN ID of the PDP.
     */
    public FrcPdp(int canId)
    {
        super(canId);

        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        pdpEnergyTaskObj = TrcTaskMgr.getInstance().createTask(moduleName + ".preContinuous", this::pdpEnergyTask);
    }   //FrcPdp

    /**
     * This method enables/disables the energy monitoring task. When the task is enabled, it also clears the
     * totalEnergyUsed array.
     *
     * @param enabled specifies true to enable the task, false to disable.
     */
    public void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%b", enabled);
        }

        if (enabled)
        {
            for (int i = 0; i < energyUsed.length; i++)
            {
                energyUsed[i] = 0.0;
            }
            lastTimestamp = TrcUtil.getCurrentTime();
            pdpEnergyTaskObj.registerTask(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
        else
        {
            pdpEnergyTaskObj.unregisterTask(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setTaskEnabled

    /**
     * This method returns the energy consumed so far by the specified channel in the unit of Watt-Hour.
     *
     * @param channel specifies the PDP channel.
     * @return energy consumed in Watt-Hour.
     */
    public double getEnergyUsed(int channel)
    {
        final String funcName = "getEnergyUsed";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "channel=%d", channel);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", energyUsed[channel]);
        }

        return energyUsed[channel];
    }   //getEnergyUsed

    //
    // Implements TrcTaskMgr.Task
    //

    /**
     * This method is called periodically to integrate the power consumption of each channel.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     */
    public void pdpEnergyTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "pdpEnergyTask";
        double currTime = TrcUtil.getCurrentTime();
        double voltage = getVoltage();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        for (int i = 0; i < energyUsed.length; i++)
        {
            energyUsed[i] += voltage*getCurrent(i)*(currTime - lastTimestamp);
        }

        lastTimestamp = currTime;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //pdpEnergyTask

}   //class FrcPdp

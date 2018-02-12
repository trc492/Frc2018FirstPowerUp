/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

package trclib;

/**
 * This class implements a platform independent Maxbotix ultrasonic sensor array. The ultrasonic sensors in the array
 * are connected in analog chain mode where only one sensor will ping at a time to eliminate cross talk. This class
 * supports both regular chain config and loop chain config.
 * https://www.maxbotix.com/documents/LV-MaxSonar-EZ_Datasheet.pdf
 */
public class TrcMaxbotixSonarArray
{
    private static final String moduleName = "TrcMaxbotixSonarArray";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static final double RANGING_START_PULSE_WIDTH = 0.02;   //in seconds
    private static final double RANGING_PERIOD = 0.05;              //in seconds

    enum State
    {
        PULL_RX_HIGH,
        PULL_RX_LOW,
        DONE
    }   //State

    private final String instanceName;
    private TrcAnalogInput[] sensors;
    private TrcDigitalOutput rx;
    private boolean loopConfig;
    private TrcStateMachine<State> sm;
    private TrcTimer timer;
    private TrcEvent event;
    private boolean autoRepeat = false;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param sensors specifies an array of Maxbotix ultrasonic sensors.
     * @param rx specifies the digital output channel the RX pin is connected to.
     * @param loopConfig specifies true if the sensor array is wired in loop configuration, false otherwise.
     */
    public TrcMaxbotixSonarArray(
            final String instanceName, TrcAnalogInput[] sensors, TrcDigitalOutput rx, boolean loopConfig)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.sensors = sensors;
        this.rx = rx;
        this.loopConfig = loopConfig;
        sm = new TrcStateMachine<>(instanceName);
        timer = new TrcTimer(instanceName);
        event = new TrcEvent(instanceName);
    }   //TrcMaxbotixSonarArray

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param sensors specifies an array of Maxbotix ultrasonic sensors.
     * @param rx specifies the digital output channel the RX pin is connected to.
     */
    public TrcMaxbotixSonarArray(final String instanceName, TrcAnalogInput[] sensors, TrcDigitalOutput rx)
    {
        this(instanceName, sensors, rx, false);
    }   //TrcMaxbotixSonarArray

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method is called to start the ranging cycle.
     *
     * @param autoRepeat specifies true to auto repeat the ranging cycle, false otherwise. autoRepeat is ignored
     *                   if the array is wired in loop config because it is already in continuous ranging mode.
     */
    public void startRanging(boolean autoRepeat)
    {
        final String funcName = "startRanging";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "autoRepeat=%s", Boolean.toString(autoRepeat));
        }

        if (!loopConfig)
        {
            this.autoRepeat = autoRepeat;
        }
        setEnabled(true);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //startRanging

    /**
     * This method is called to start the ranging cycle.
     */
    public void startRanging()
    {
        startRanging(false);
    }   //startRanging

    /**
     * This method is called to stop the ranging cycle. Ranging cycle can only be stopped if it is not wired in
     * loop config and was set to autoRepeat mode. If the sensor array is wired in loop config, the only way to
     * stop ranging is to remove power.
     */
    public void stopRanging()
    {
        final String funcName = "stopRanging";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (!loopConfig && autoRepeat)
        {
            autoRepeat = false;
            setEnabled(false);
        }
    }   //stopRanging

    /**
     * This method returns the distance data of the specifies sensor index.
     *
     * @param sensorIndex specifies the index of the ultrasonic sensor to read its distance data.
     * @return distance data of the specified sensor.
     */
    public TrcSensor.SensorData<Double> getDistance(int sensorIndex)
    {
        return sensors[sensorIndex].getData(0);
    }   //getDistance

    /**
     * This method is called to start the task that generates the RX pulse for ranging to start.
     *
     * @param enabled specifies true to start the task, false otherwise.
     */
    private void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "enabled=%s", Boolean.toString(enabled));
        }

        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        if (enabled)
        {
            taskMgr.registerTask(instanceName, this::preContinuousTask, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
            sm.start(State.PULL_RX_HIGH);
        }
        else
        {
            sm.stop();
            taskMgr.unregisterTask(this::preContinuousTask, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //setEnabled

    //
    // Implements TrcTaskMgr.Task interface.
    //

    /**
     * This method is called periodically to run the state machine that generates teh RX pulse.
     *
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     */
    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "preContinuousTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "runMode=%s", runMode);
        }

        if (sm.isReady())
        {
            State state = sm.getState();

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "State: %s", state);
            }

            switch (state)
            {
                case PULL_RX_HIGH:
                    //
                    // Set RX high for PULSE_WIDTH time.
                    //
                    rx.setState(true);
                    timer.set(RANGING_START_PULSE_WIDTH, event);
                    sm.waitForSingleEvent(event, State.PULL_RX_LOW);
                    break;

                case PULL_RX_LOW:
                    rx.setState(false);
                    timer.set(RANGING_PERIOD*sensors.length, event);
                    sm.waitForSingleEvent(event, !loopConfig && autoRepeat? State.PULL_RX_HIGH: State.DONE);
                    break;

                case DONE:
                    setEnabled(false);
                    break;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //preContinuousTask

}   //class TrcMaxbotixSonarArray

/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import edu.wpi.first.wpilibj.Solenoid;
import trclib.TrcBooleanState;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcTimer;
import trclib.TrcUtil;

public class FrcPneumatic
{
    private static final String moduleName = "FrcPneumatic";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public class SolenoidAction
    {
        public byte onSolMask;
        public double onPeriod;
    }   //class SolenoidAction

    private enum State
    {
        START,
        DONE
    }   //enum State
    
    private String instanceName;
    private TrcTaskMgr.TaskObject stopTaskObj;
    private TrcTaskMgr.TaskObject postContinuousTaskObj;
    private TrcStateMachine<State> solSM;
    private TrcTimer solTimer;
    private TrcEvent timerEvent;
    private SolenoidAction[] pulseActions = new SolenoidAction[3];
    private SolenoidAction[] actionList;
    private boolean repeatActions;
    private TrcEvent notifyEvent;
    private int actionIndex;
    private int numActions;
    private Solenoid[] solenoids;
    private TrcBooleanState solenoidState;

    private void initPneumatic(final String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        stopTaskObj = taskMgr.createTask(instanceName + ".stop", this::stopTask);
        postContinuousTaskObj = taskMgr.createTask(instanceName + ".postContinuous", this::postContinuousTask);
        solSM = new TrcStateMachine<>(instanceName);
        solTimer = new TrcTimer(instanceName);
        timerEvent = new TrcEvent(instanceName);
        for (int i = 0; i < pulseActions.length; i++)
        {
            pulseActions[i] = new SolenoidAction();
        }
        actionList = null;
        repeatActions = false;
        notifyEvent = null;
        actionIndex = 0;
        numActions = 0;
        if (solenoids.length <= 2)
        {
            solenoidState = new TrcBooleanState(instanceName, false);
            retract();
        }
        else
        {
            solenoidState = null;
        }
    }   //initPneumatic

    public FrcPneumatic(final String instanceName, final int module, final int channel)
    {
        solenoids = new Solenoid[1];
        solenoids[0] = new Solenoid(module, channel);
        initPneumatic(instanceName);
    }   //FrcPneumatic

    public FrcPneumatic(final String instanceName, final int module, final int channel1, final int channel2)
    {
        solenoids = new Solenoid[2];
        solenoids[0] = new Solenoid(module, channel1);
        solenoids[1] = new Solenoid(module, channel2);
        initPneumatic(instanceName);
    }   //FrcPneumatic

    public FrcPneumatic(
        final String instanceName, final int module, final int channel1, final int channel2, final int channel3)
    {
        solenoids = new Solenoid[3];
        solenoids[0] = new Solenoid(module, channel1);
        solenoids[1] = new Solenoid(module, channel2);
        solenoids[2] = new Solenoid(module, channel3);
        initPneumatic(instanceName);
    }   //FrcPneumatic

    public FrcPneumatic(final String instanceName, final int module, int[] channels)
    {
        solenoids = new Solenoid[channels.length];
        for (int i = 0; i < solenoids.length; i++)
        {
            solenoids[i] = new Solenoid(module, channels[i]);
        }
        initPneumatic(instanceName);
    }   //FrcPneumatic

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    public void set(byte bitMask, boolean on)
    {
        final String funcName = "set";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "bitMask=%x,on=%s",
                bitMask, Boolean.toString(on));
        }

        cancel();
        for (int i = 0; i < solenoids.length; i++)
        {
            if (((1 << i) & bitMask) != 0)
            {
                solenoids[i].set(on);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //set

    public void set(byte onMask, byte offMask)
    {
        set(offMask, false);
        set(onMask, true);
    }   //set

    public void set(byte onMask, double onPeriod, TrcEvent event)
    {
        pulseActions[0].onSolMask = onMask;
        pulseActions[0].onPeriod = onPeriod;
        pulseActions[1].onSolMask = 0;
        pulseActions[1].onPeriod = 0.0;
        set(pulseActions, 2, false, event);
    }   //set

    public void set(
            byte on1Mask,
            double on1Period,
            byte on2Mask,
            double on2Period,
            boolean repeat,
            TrcEvent event)
    {
        pulseActions[0].onSolMask = on1Mask;
        pulseActions[0].onPeriod = on1Period;
        pulseActions[1].onSolMask = on2Mask;
        pulseActions[1].onPeriod = on2Period;
        pulseActions[2].onSolMask = 0;
        pulseActions[2].onPeriod = 0.0;
        set(pulseActions, 3, repeat, event);
    }   //set

    public void set(
            SolenoidAction[] actionList,
            int numActions,
            boolean repeat,
            TrcEvent event)
    {
        final String funcName = "set";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "numActions=%d,repeat=%s,event=%s",
                    numActions, Boolean.toString(repeat),
                    event != null? event.toString(): "null");
        }

        if (actionList.length > 0 && actionList[0].onPeriod > 0.0)
        {
            cancel();
            this.actionList = actionList;
            this.numActions = numActions;
            if (numActions > actionList.length)
            {
                this.numActions = actionList.length;
            }
            this.repeatActions = repeat;
            if (event != null)
            {
                event.clear();
            }
            this.notifyEvent = event;
            actionIndex = 0;
            setTaskEnabled(true);
            solSM.start(State.START);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //set

    public void setState(boolean state, boolean toggleMode)
    {
        final String funcName = "setState";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "state=%s,toggle=%s",
                    Boolean.toString(state), Boolean.toString(toggleMode));
        }

        if (solenoids.length <= 2)
        {
            if (toggleMode)
            {
                if (state)
                {
                    if (solenoidState.toggleState())
                    {
                        extend();
                    }
                    else
                    {
                        retract();
                    }
                }
            }
            else if (state)
            {
                extend();
            }
            else
            {
                retract();
            }
        }
        else
        {
            throw new UnsupportedOperationException(
                    "Method supports only one or two-valve cylinders.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setState

    public void setState(boolean state)
    {
        setState(state, false);
    }   //setState

    public void extend()
    {
        final String funcName = "extend";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (solenoids.length == 2)
        {
            //
            // Two-valve cylinder: first channel is the extend valve and
            // second channel is the retract valve.
            //
            set((byte)(1 << 0), (byte)(1 << 1));
        }
        else if (solenoids.length == 1)
        {
            //
            // One-valve spring loaded cylinder: only one extend channel.
            //
            set((byte)1, true);
        }
        else
        {
            throw new UnsupportedOperationException(
                    "Method supports only one or two-valve cylinders.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //extend

    public void extend(double period, TrcEvent event)
    {
        final String funcName = "extend";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "period=%f,event=%s",
                    period, event != null? event.toString(): "null");
        }

        if (solenoids.length == 1 || solenoids.length == 2)
        {
            set((byte)(1 << 0), period, event);
        }
        else
        {
            throw new UnsupportedOperationException(
                    "Method supports only one or two-valve cylinders.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //extend

    public void retract()
    {
        final String funcName = "retract";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (solenoids.length == 2)
        {
            //
            // Two-valve cylinder: first channel is the extend valve and
            // second channel is the retract valve.
            //
            set((byte)(1 << 1), (byte)(1 << 0));
        }
        else if (solenoids.length == 1)
        {
            //
            // One-valve spring loaded cylinder: only one extend channel.
            //
            set((byte)1, false);
        }
        else
        {
            throw new UnsupportedOperationException(
                    "Method supports only one or two-valve cylinders.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //retract

    public void retract(double period, TrcEvent event)
    {
        final String funcName = "retract";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "period=%f,event=%s",
                    period, event != null? event.toString(): "null");
        }

        if (solenoids.length == 2)
        {
            //
            // Two-valve cylinder: first channel is the extend valve and
            // second channel is the retract valve.
            //
            set((byte)(1 << 1), period, event);
        }
        else
        {
            throw new UnsupportedOperationException(
                    "Method supports only two-valve cylinders.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //extend

    public void timedExtend(double extendPeriod, double retractPeriod, TrcEvent event)
    {
        final String funcName = "timedExtend";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "extendPeriod=%.3f,retractPeriod=%.3f,event=%s",
                extendPeriod, retractPeriod, event);
        }

        if (solenoids.length == 2)
        {
            //
            // Two-valve cylinder: first channel is the extend valve and
            // second channel is the retract valve.
            //
            set((byte)(1 << 0), extendPeriod, (byte)(1 << 1), retractPeriod, false, event);
        }
        else
        {
            throw new UnsupportedOperationException("Method supports only two-valve cylinders.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //timedExtend

    public void timedExtend(double extendPeriod)
    {
        timedExtend(extendPeriod, 0.0, null);
    }   //timedExtend

    public boolean isExtended()
    {
        final String funcName = "isExtended";
        boolean state = false;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (solenoids.length <= 2)
        {
            state = solenoidState.getState();
        }
        else
        {
            throw new UnsupportedOperationException("Method supports only two-valve cylinders.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", state);
        }

        return state;
    }   //isExtended

    private void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "enabled=%b", enabled);
        }

        if (enabled)
        {
            stopTaskObj.registerTask(TrcTaskMgr.TaskType.STOP_TASK);
            postContinuousTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            stopTaskObj.unregisterTask(TrcTaskMgr.TaskType.STOP_TASK);
            postContinuousTaskObj.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //setTaskEnabled

    private void cancel()
    {
        final String funcName = "cancel";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        if (solSM.isEnabled())
        {
            setTaskEnabled(false);
            solTimer.cancel();
            solSM.stop();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //cancel

    //
    // Implements TrcTaskMgr.Task
    //

    public void stopTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "stopTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        set((byte)0xff, false);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //stopTask

    public void postContinuousTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "postContinuous";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        if (solSM.isReady())
        {
            State state = solSM.getState();
            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Executing state %d.", state);
            }

            switch (state)
            {
            case START:
                if (actionIndex < numActions)
                {
                    //
                    // Turn the each of the solenoids ON/OFF.
                    //
                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(
                                funcName,
                                "[%f] Executing action %d/%d",
                                TrcUtil.getCurrentTime(),
                                actionIndex, numActions);
                    }
                    for (int i = 0; i < solenoids.length; i++)
                    {
                        if (((1 << i) & actionList[actionIndex].onSolMask) != 0)
                        {
                            solenoids[i].set(true);
                            if (debugEnabled)
                            {
                                dbgTrace.traceInfo(
                                        funcName, "Set solenoid %d ON.", i);
                            }
                        }
                        else
                        {
                            solenoids[i].set(false);
                            if (debugEnabled)
                            {
                                dbgTrace.traceInfo(
                                        funcName, "Set solenoid %d OFF.", i);
                            }
                        }
                    }
                    //
                    // Set timer and wait for it if necessary.
                    //
                    if (actionList[actionIndex].onPeriod > 0.0)
                    {
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(
                                    funcName,
                                    "Set timer for %f",
                                    actionList[actionIndex].onPeriod);
                        }
                        solTimer.set(
                                actionList[actionIndex].onPeriod,
                                timerEvent);
                        solSM.addEvent(timerEvent);
                        solSM.waitForEvents(state);
                    }
                    //
                    // Move to the next action.
                    //
                    actionIndex++;
                    if (repeatActions && actionIndex >= numActions)
                    {
                        actionIndex = 0;
                    }
                }
                else
                {
                    //
                    // No more action, we are done.
                    //
                    solSM.setState(State.DONE);;
                }
                break;

            case DONE:
            default:
                //
                // We are done.
                //
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Done!");
                }
                setTaskEnabled(false);
                if (notifyEvent != null)
                {
                    notifyEvent.set(true);
                    notifyEvent = null;
                }
                actionList = null;
                repeatActions = false;
                actionIndex = 0;
                numActions = 0;
                solSM.stop();
                break;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //PostContinuousTask

}   //class FrcPneumatic

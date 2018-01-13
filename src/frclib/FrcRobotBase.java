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

import java.io.InputStream;
import java.io.IOException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType; 
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import hallib.HalDashboard;
import hallib.HalDbgLog;
import trclib.TrcDbgTrace;
import trclib.TrcRobot.*;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

/**
 * This class defines and implements the FrcRobotBase object. The FrcRobotBase object implements a cooperative
 * multitasking robot. Different subsystems register themselves as CoopTasks. FrcRobotBase uses the TaskMgr to
 * task switch between different subsystem tasks at various points in the robot loop. This basically simulates
 * a cooperative multitasking scheduler that task switches between them in different modes.
 */
public abstract class FrcRobotBase extends RobotBase
{
    private static final String moduleName = "FrcRobotBase";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static final boolean dashboardEnabled = true;

    /**
     * This method is called to initialize the robot.
     */
    public abstract void robotInit();

    /**
     * This method is called to prepare the robot before a robot mode is about to start.
     */
    public abstract void robotStartMode();

    /**
     * This method is called to prepare the robot right after a robot mode has been stopped.
     */
    public abstract void robotStopMode();

    private TrcTaskMgr taskMgr = new TrcTaskMgr();
    private HalDashboard dashboard = new HalDashboard();

    private static FrcRobotBase instance;
    private final String progName;
    private RobotMode teleOpMode = null;
    private RobotMode autoMode = null;
    private RobotMode testMode = null;
    private RobotMode disabledMode = null;
    private static double modeStartTime = 0.0;
    private static double modeElapsedTime = 0.0;
    private static long loopCounter = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param progName specifies the program name.
     */
    public FrcRobotBase(final String progName)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        FrcRobotBase.instance = this;
        this.progName = progName;
        dashboard.clearDisplay();
    }   //FrcRobotBase

    /**
     * This method returns the saved instance. This is a static method. So other class can get to this class instance
     * by calling getInstance(). This is very useful for other classes that need to access the public fields and
     * methods.
     *
     * @return save instance of this class.
     */
    public static FrcRobotBase getInstance()
    {
        return instance;
    }   //getInstance

    /**
     * This method returns the elapsed time since the robot mode starts. This is the elapsed time after
     * RobotMode.startMode() is called.
     *
     * @return robot mode elapsed time in seconds.
     */
    public static double getModeElapsedTime()
    {
        modeElapsedTime = TrcUtil.getCurrentTime() - modeStartTime;
        return modeElapsedTime;
    }   //getModeElapsedTime

    /**
     * This method returns the loop counter. This is very useful for code to determine if it is called multiple times
     * in the same loop. For example, it can be used to optimize sensor access so that if the sensor is accessed in
     * the same loop, there is no reason to create a new bus transaction to get "fresh" data from the sensor.
     *
     * @return loop counter value.
     */
    public static long getLoopCounter()
    {
        return loopCounter;
    }   //getLoopCounter

    /**
     * This method is called by the subclass to set up various robot mode objects.
     *
     * @param teleOpMode specifies the TeleOp mode object.
     * @param autoMode specifies the Autonomous mode object.
     * @param testMode specifies the Test mode object.
     * @param disabledMode specifies the Disabled mode object.
     */
    public void setupRobotModes(RobotMode teleOpMode, RobotMode autoMode, RobotMode testMode, RobotMode disabledMode)
    {
        this.teleOpMode = teleOpMode;
        this.autoMode = autoMode;
        this.testMode = testMode;
        this.disabledMode = disabledMode;
    }   //setupRobotModes

    /**
     * Start the competition match. This specific startCompetition() implements "main loop" behavior like that of
     * the FRC control system in 2008 and earlier, with a primary (slow) loop that is called periodically, and a
     * "fast loop" (a.k.a. "spin loop") that is called as fast as possible with no delay between calls. This code
     * needs to track the order of the modes starting to ensure that everything happens in the right order. Repeatedly
     * run the correct method, either Autonomous or TeleOp when the robot is enabled. After running the correct method,
     * wait for some state to change, either the other mode starts or the robot is disabled. Then go back and wait for
     * the robot to be enabled again.
     */
    public void startCompetition()
    {
        final String funcName = "startCompetition";

        System.out.printf(
            HalDbgLog.ESC_PREFIX + HalDbgLog.SGR_FG_BLACK +
            HalDbgLog.ESC_SEP + HalDbgLog.SGR_BG_WHITE +
            HalDbgLog.ESC_SUFFIX +
            "\n****************************************\n" +
            "Host Name: %s\n" +
            "  Program: %s\n"+
            "\n****************************************\n" +
            HalDbgLog.ESC_NORMAL,
            getHostName(), progName);

        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Iterative);

        robotInit();

        //
        // Tell the DS that the robot is ready to be enabled.
        //
        HAL.observeUserProgramStarting();

        LiveWindow.setEnabled(false);
        //
        // loop forever, calling the appropriate mode-dependent function
        //
        final double timesliceThreshold = 0.05;
        RunMode prevMode = RunMode.INVALID_MODE;
        RunMode currMode = RunMode.INVALID_MODE;

        while (true)
        {
            double timeSliceStart = Timer.getFPGATimestamp();

            prevMode = currMode;

            //
            // Determine the current run mode.
            //
            if (isDisabled())
            {
                currMode = RunMode.DISABLED_MODE;
            }
            else if (isTest())
            {
                currMode = RunMode.TEST_MODE;
            }
            else if (isAutonomous())
            {
                currMode = RunMode.AUTO_MODE;
            }
            else if (isOperatorControl())
            {
                currMode = RunMode.TELEOP_MODE;
            }
            else
            {
                currMode = RunMode.INVALID_MODE;
            }

            if (currMode != prevMode)
            {
                //
                // Detected mode transition.
                //
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Mode Transition: %s->%s.", prevMode.toString(), currMode.toString());
                }

                //
                // Execute all stop tasks for previous mode.
                //
                if (prevMode != RunMode.INVALID_MODE)
                {
                    taskMgr.executeTaskType(TrcTaskMgr.TaskType.STOP_TASK, prevMode);
                }

                //
                // Stop previous mode.
                // 
                if (prevMode == RunMode.DISABLED_MODE && disabledMode != null)
                {
                    disabledMode.stopMode();
                }
                else if (prevMode == RunMode.TEST_MODE && testMode != null)
                {
                    testMode.stopMode();
                }
                else if (prevMode == RunMode.AUTO_MODE && autoMode != null)
                {
                    autoMode.stopMode();
                }
                else if (prevMode == RunMode.TELEOP_MODE && teleOpMode != null)
                {
                    teleOpMode.stopMode();
                }
                robotStopMode();

                //
                // Start current mode.
                //
                modeStartTime = TrcUtil.getCurrentTime();
                robotStartMode();
                if (currMode == RunMode.DISABLED_MODE)
                {
                    LiveWindow.setEnabled(false);
                    if (disabledMode != null)
                    {
                        disabledMode.startMode();
                    }
                }
                else if (currMode == RunMode.TEST_MODE)
                {
//                    LiveWindow.setEnabled(true);
                    if (testMode != null)
                    {
                        testMode.startMode();
                    }
                }
                else if (currMode == RunMode.AUTO_MODE)
                {
                    LiveWindow.setEnabled(false);
                    if (autoMode != null)
                    {
                        autoMode.startMode();
                    }
                }
                else if (currMode == RunMode.TELEOP_MODE)
                {
                    LiveWindow.setEnabled(false);
                    if (teleOpMode != null)
                    {
                        teleOpMode.startMode();
                    }
                }

                //
                // Execute all start tasks for current mode.
                //
                if (currMode != RunMode.INVALID_MODE)
                {
                    taskMgr.executeTaskType(TrcTaskMgr.TaskType.START_TASK, currMode);
                }
            }

            modeElapsedTime = TrcUtil.getCurrentTime() - modeStartTime;
            if (nextPeriodReady())
            {
                //
                // Run periodic mode.
                //
                taskMgr.executeTaskType(TrcTaskMgr.TaskType.PREPERIODIC_TASK, currMode);
                if (currMode == RunMode.DISABLED_MODE)
                {
                    HAL.observeUserProgramDisabled();
                    if (disabledMode != null)
                    {
                        disabledMode.runPeriodic(modeElapsedTime);
                    }
                }
                else if (currMode == RunMode.TEST_MODE)
                {
                    HAL.observeUserProgramTest();
                    if (testMode != null)
                    {
                        testMode.runPeriodic(modeElapsedTime);
                    }
                }
                else if (currMode == RunMode.AUTO_MODE)
                {
                    HAL.observeUserProgramAutonomous();
                    if (autoMode != null)
                    {
                        autoMode.runPeriodic(modeElapsedTime);
                    }
                }
                else if (currMode == RunMode.TELEOP_MODE)
                {
                    HAL.observeUserProgramTeleop();
                    if (teleOpMode != null)
                    {
                        teleOpMode.runPeriodic(modeElapsedTime);
                    }
                }

                //
                // Run post periodic tasks.
                //
                taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTPERIODIC_TASK, currMode);
            }

            //
            // Run continuous mode.
            //
            taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, currMode);
            if (currMode == RunMode.DISABLED_MODE && disabledMode != null)
            {
                disabledMode.runContinuous(modeElapsedTime);
            }
            else if (currMode == RunMode.TEST_MODE && testMode != null)
            {
                testMode.runContinuous(modeElapsedTime);
            }
            else if (currMode == RunMode.AUTO_MODE && autoMode != null)
            {
                autoMode.runContinuous(modeElapsedTime);
            }
            else if (currMode == RunMode.TELEOP_MODE && teleOpMode != null)
            {
                teleOpMode.runContinuous(modeElapsedTime);
            }
            taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK, currMode);

            if (dashboardEnabled)
            {
                dashboard.displayPrintf(0, "[%3d:%06.3f] %s",
                    (int)(modeElapsedTime/60), modeElapsedTime%60, currMode.toString());
            }

            //
            // Do house keeping statistics.
            //
            double timeSliceUsed = Timer.getFPGATimestamp() - timeSliceStart;
            if (timeSliceUsed > timesliceThreshold)
            {
                TrcDbgTrace.getGlobalTracer().traceWarn(funcName, "%s takes too long (%5.3fs)\n",
                    currMode.toString(), timeSliceUsed);
            }
        }
    }   //startCompetition

    /**
     * This method returns the host name of the RobotRIO.
     *
     * @return host name.
     */
    private String getHostName()
    {
        String hostName = null;

        try
        {
            byte[] buff = new byte[256];
            Process proc = Runtime.getRuntime().exec("hostname");
            InputStream inStream = proc.getInputStream();
            inStream.read(buff, 0, buff.length);
            hostName = new String(buff);
        }
        catch(IOException e)
        {
            e.printStackTrace();
        }

        return hostName;
    }   //getHostName

    /**
     * Determine if the appropriate next periodic function should be called. Call the periodic functions whenever
     * a packet is received from the Driver Station or about every 20 msec.
     */
    private boolean nextPeriodReady()
    {
        return m_ds.isNewControlData();
    }   //nextPeriodReady

}   //class FrcRobotBase

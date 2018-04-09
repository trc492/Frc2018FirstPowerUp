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

package team492;

import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;
import trclib.TrcRobot;
import trclib.TrcRobot.RunMode;

public class FrcDisabled implements TrcRobot.RobotMode
{
    private static final String moduleName = "FrcDisabled";

    private enum State
    {
        START,
        AUTO_DONE,
        TELEOP_DONE,
        DONE
    }

    private Robot robot;
    private State state = State.START;
    private boolean autoFinished = false;

    public FrcDisabled(Robot robot)
    {
        this.robot = robot;
    } // FrcDisabled

    //
    // Implements TrcRobot.RunMode interface.
    //

    @Override
    public void startMode()
    {
        final String funcName = moduleName + ".startMode";

        if (robot.ds.isFMSAttached() || autoFinished)
        {
            //
            // We are in a competition match or auto just got done.
            //
            switch (state)
            {
                case START:
                    autoFinished = false;
                    // Disabled periodic will monitor and transition to AUTO_DONE.
                    break;

                case AUTO_DONE:
                    state = State.TELEOP_DONE;
                    autoFinished = true;
                    break;

                case TELEOP_DONE:
                    // TODO: Figure out what the hell is going on here
                    try
                    {
                        String traceLogName = robot.globalTracer.getTraceLogName();
                        String prefix = traceLogName.substring(0, traceLogName.indexOf('!'));
                        String newFile = String.format("%s!%s_%s%03d.log",
                            prefix, robot.eventName, robot.matchType, robot.matchNumber);
                        robot.setTraceLogEnabled(true);
                        robot.globalTracer.traceInfo(funcName, "### OldName: %s, NewName: %s ###",
                            traceLogName, newFile);
                        robot.setTraceLogEnabled(false);
                        File file = new File(traceLogName);
                        robot.closeTraceLog();
                        file.renameTo(new File(newFile));
                    }
                    catch(Exception e)
                    {
                        // We failed to rename the file, close the log anyway.
                        robot.closeTraceLog();
                        DriverStation.reportError(e.getMessage(), false);
                        // Fail silently
                    }
                    state = State.DONE;
                    autoFinished = false;
                    break;

                default:
                case DONE:
                    break;
            }
        }
        else
        {
            //
            // Not in competition match, probably in test or practice mode.
            //
            robot.closeTraceLog();
        }
    } // startMode

    @Override
    public void stopMode()
    {
        if (!robot.ds.isFMSAttached())
        {
            robot.openTraceLog();
        }
    } // stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        if (!robot.traceLogOpened && state != State.DONE && robot.ds.isFMSAttached())
        {
            robot.openTraceLog();
            state = State.AUTO_DONE;
        }

        robot.updateDashboard(RunMode.DISABLED_MODE);
        robot.announceIdling();
    } // runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
    } // runContinuous

} // class FrcDisabled

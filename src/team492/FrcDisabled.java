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

import trclib.TrcRobot;
import trclib.TrcRobot.RunMode;

public class FrcDisabled implements TrcRobot.RobotMode
{
    private enum State
    {
        START,
        ENTER_RUN_MODE,
        EXIT_RUN_MODE,
        EXIT_TELEOP_MODE
    }

    private Robot robot;
    private State state;

    public FrcDisabled(Robot robot)
    {
        this.robot = robot;
        state = State.START;
    } // FrcDisabled

    //
    // Implements TrcRobot.RunMode interface.
    //

    @Override
    public void startMode(RunMode prevMode)
    {
        switch (state)
        {
            case START:
                robot.openTraceLog();
                state = State.ENTER_RUN_MODE;
                break;

            case EXIT_RUN_MODE:
                // Exiting test or practice mode, close the log with appropriate mode name.
                robot.closeTraceLog(prevMode.toString() + ".log");
                state = State.START;
                break;

            case EXIT_TELEOP_MODE:
                // Exiting competition, close the log with proper FMS info as the log file name.
                robot.closeTraceLog(
                    String.format("%s_%s%03d.log", robot.eventName, robot.matchType, robot.matchNumber));
                state = State.START;
                break;

            default:
                break;
        }
    } // startMode

    @Override
    public void stopMode(RunMode nextMode)
    {
        switch (state)
        {
            case ENTER_RUN_MODE:
                if (nextMode == RunMode.AUTO_MODE && robot.ds.isFMSAttached())
                {
                    // Going into competition match, get FMS info.
                    robot.getFMSInfo();
                    state = State.EXIT_TELEOP_MODE;
                }
                else
                {
                    // Going into test or practice mode.
                    state = State.EXIT_RUN_MODE;
                }
                break;

            default:
                break;
        }
    } // stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        robot.updateDashboard(RunMode.DISABLED_MODE);
        robot.announceIdling();
    } // runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
    } // runContinuous

} // class FrcDisabled

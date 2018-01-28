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

import trclib.TrcEvent;
import trclib.TrcPidDrive.TurnMode;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

class CmdWaltzTurn implements TrcRobot.RobotCommand
{
    private static enum State
    {
        WALTZ_TURN,
        DONE
    }   //enum State

    private static final String moduleName = "CmdWaltzTurn";

    private Robot robot;
    private boolean clockwiseTurn = false;
    private boolean driveInverted = false;

    private TrcEvent event;
    private TrcStateMachine<State> sm;
    private TurnMode prevTurnMode;

    CmdWaltzTurn(Robot robot)
    {
        this.robot = robot;

        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        prevTurnMode = robot.pidDrive.getTurnMode();
    }   //CmdWaltzTurn

    public void setClockwiseTurn(boolean clockwiseTurn, boolean driveInverted)
    {
        this.clockwiseTurn = clockwiseTurn;
        this.driveInverted = driveInverted;
    }   //setClockwiseTurn

    public void start()
    {
        stop();
        sm.start(State.WALTZ_TURN);
    }

    public void stop()
    {
        if (robot.pidDrive.isActive())
        {
            robot.pidDrive.cancel();
            robot.pidDrive.setTurnMode(prevTurnMode);
        }
    }   //cancel

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();
        //
        // Print debug info.
        //
        State state = sm.getState();
        robot.dashboard.displayPrintf(1, "State: %s", state != null? state.toString(): "Disabled");

        if (sm.isReady())
        {
            state = sm.getState();

            switch (state)
            {
                case WALTZ_TURN:
                    //
                    // Do the waltz turn.
                    //
                    robot.targetHeading = robot.driveBase.getHeading();
                    robot.targetHeading += clockwiseTurn? 180.0: -180.0;

                    prevTurnMode = robot.pidDrive.getTurnMode();
                    robot.pidDrive.setTurnMode(driveInverted? TurnMode.PIVOT_FORWARD: TurnMode.PIVOT_BACKWARD);
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event, 2.0);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    robot.pidDrive.setTurnMode(prevTurnMode);
                    done = true;
                    sm.stop();
                    break;
            }

            robot.traceStateInfo(elapsedTime, state.toString());
        }

        return done;
    }   //cmdPeriodic

}   //class CmdWaltzTurn


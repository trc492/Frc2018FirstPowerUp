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

package common;

import team492.Robot;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidController.PidCoefficients;
import trclib.TrcPidDrive;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class CmdPidDrive implements TrcRobot.RobotCommand
{
    private static enum State
    {
        DO_DELAY,
        DO_PID_DRIVE,
        DONE
    }   //enum State

    private static final String moduleName = "CmdPidDrive";

    private Robot robot;
    private TrcPidDrive pidDrive;
    private TrcPidController xPidCtrl;
    private TrcPidController yPidCtrl;
    private TrcPidController turnPidCtrl;

    private double delay;
    private double xDistance;
    private double yDistance;
    private double heading;
    private double drivePowerLimit;
    private boolean testMode;

    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    public CmdPidDrive(
        Robot robot,
        TrcPidDrive pidDrive, TrcPidController xPidCtrl, TrcPidController yPidCtrl, TrcPidController turnPidCtrl,
        double delay, double xDistance, double yDistance, double heading, double drivePowerLimit,
        boolean testMode)
    {
        this.robot = robot;
        this.pidDrive = pidDrive;
        this.xPidCtrl = xPidCtrl;
        this.yPidCtrl = yPidCtrl;
        this.turnPidCtrl = turnPidCtrl;
        this.delay = delay;
        this.xDistance = xDistance;
        this.yDistance = yDistance;
        this.heading = heading;
        this.drivePowerLimit = drivePowerLimit;
        this.testMode = testMode;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);

        robot.tracer.traceInfo(
            moduleName, "pidDrive=%s, xPidCtrl=%s, yPidCtrl=%s, turnPidCtrl=%s",
            pidDrive, xPidCtrl, yPidCtrl, turnPidCtrl);
        robot.tracer.traceInfo(
            moduleName, "delay=%.3f, xDist=%.1f, yDist=%.1f, heading=%.1f, powerLimit=%.1f, testMode=%b",
            delay, xDistance, yDistance, heading, drivePowerLimit, testMode);
    }   //CmdPidDrive

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

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
                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(State.DO_PID_DRIVE);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.DO_PID_DRIVE);
                    }
                    break;

                case DO_PID_DRIVE:
                    //
                    // Drive the set distance and heading.
                    //
                    if (testMode)
                    {
                        if (xPidCtrl != null && xDistance != 0.0)
                        {
                            xPidCtrl.setPidCoefficients(
                                new PidCoefficients(robot.tuneKp, robot.tuneKi, robot.tuneKd, 0.0));
                        }
                        else if (yPidCtrl != null && yDistance != 0.0)
                        {
                            yPidCtrl.setPidCoefficients(
                                new PidCoefficients(robot.tuneKp, robot.tuneKi, robot.tuneKd, 0.0));
                        }
                        else if (turnPidCtrl != null && heading != 0.0)
                        {
                            robot.gyroTurnPidCtrl.setPidCoefficients(
                                new PidCoefficients(robot.tuneKp, robot.tuneKi, robot.tuneKd, 0.0));
                        }
                    }

                    if (xPidCtrl != null && xDistance != 0.0)
                        xPidCtrl.setOutputRange(-drivePowerLimit, drivePowerLimit);
                    if (yPidCtrl != null && yDistance != 0.0)
                        yPidCtrl.setOutputRange(-drivePowerLimit, drivePowerLimit);
                    if (turnPidCtrl != null && heading != 0.0)
                        turnPidCtrl.setOutputRange(-drivePowerLimit, drivePowerLimit);

                    pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    if (xPidCtrl != null) xPidCtrl.setOutputRange(-1.0, 1.0);
                    if (yPidCtrl != null) yPidCtrl.setOutputRange(-1.0, 1.0);
                    if (turnPidCtrl != null) turnPidCtrl.setOutputRange(-1.0, 1.0);
                    done = true;
                    sm.stop();
                    break;
            }

            robot.traceStateInfo(elapsedTime, state.toString());
        }

        return done;
    }   //cmdPeriodic

}   //class CmdPidDrive

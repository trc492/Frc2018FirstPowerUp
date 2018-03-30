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

/**
 * This class implements a generic PID control drive command. It is agnostic to the PID controller sensors.
 * The caller provides the PID drive object as well as all PID controllers which means the caller controls
 * what sensors are controlling the X, Y and turn PID controllers. For example, the caller can provide a PID
 * drive object that uses the encoders to control the X and Y PID controllers and a gyro for the turn PID
 * controller. The caller can also use the encoders to control the X and Y PID controllers but a camera to
 * control the turn PID controller.
 */
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

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param pidDrive specifies the PID drive object to be used for PID controlled drive.
     * @param xPidCtrl specifies the PID controller for the X direction.
     * @param yPidCtrl specifies the PID controller for the Y direction.
     * @param turnPidCtrl specifies the PID controller for the turn.
     * @param delay specifies delay in seconds before PID drive starts. 0 means no delay.
     * @param xDistance specifies the target distance for the X direction.
     * @param yDistance specifies the target distance for the Y direction.
     * @param heading specifies the target heading.
     * @param drivePowerLimit specifies the power limit to be applied for the PID controlled drive.
     * @param testMode specifies true if in test mode which allows getting PID constants from the robot object
     *        for PID tuning, false otherwise.
     */
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

        robot.globalTracer.traceInfo(
            moduleName, "pidDrive=%s, xPidCtrl=%s, yPidCtrl=%s, turnPidCtrl=%s",
            pidDrive, xPidCtrl, yPidCtrl, turnPidCtrl);
        robot.globalTracer.traceInfo(
            moduleName, "delay=%.3f, xDist=%.1f, yDist=%.1f, heading=%.1f, powerLimit=%.1f, testMode=%b",
            delay, xDistance, yDistance, heading, drivePowerLimit, testMode);
    }   //CmdPidDrive

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();

        if (done) return true;

        State state = sm.checkReadyAndGetState();

        //
        // Print debug info.
        //
        robot.dashboard.displayPrintf(1, "State: %s", state == null? "NotReady": state);

        if (state != null)
        {
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
                        //
                        // We are in test mode, modify the PID constants from values stored in the Robot class.
                        //
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
                    //
                    // Set power limits for each direction if applicable.
                    //
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

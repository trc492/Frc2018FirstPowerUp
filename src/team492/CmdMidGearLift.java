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

package team492;

import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdMidGearLift implements TrcRobot.RobotCommand
{
    private static enum State
    {
        DO_DELAY,
        GO_TO_MID_LIFT,
        VISION_GEAR_DEPLOY,
        GO_TO_SIDEWALL,
        GO_TO_LOADING_STATION,
        DONE
    }   //enum State

    private static final String moduleName = "CmdMidGearLift";

    private Robot robot;
    private double delay;

    private double midLiftDistance;
    private double midSidewallDistance;
    private double midLoadingStationDistance;

    private CmdVisionGearDeploy cmdVisionDeploy;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    CmdMidGearLift(Robot robot, double delay)
    {
        this.robot = robot;
        this.delay = delay;

        //
        // Get parameters from the SmartDashboard. 
        //

        midLiftDistance = HalDashboard.getNumber("MidLiftDistance", 0.0);
        midSidewallDistance = Math.abs(HalDashboard.getNumber("MidSidewallDistance", 75.0));
        midLoadingStationDistance = HalDashboard.getNumber("MidLoadingStationDistance", 30.0*12.0);

        cmdVisionDeploy = new CmdVisionGearDeploy(robot);

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);

        robot.tracer.traceInfo(
            moduleName, "delay=%.3f, alliance=%s, liftDist=%.1f, sidewallDist=%.1f, loadingStationDist=%.1f",
            delay, robot.alliance.toString(), midLiftDistance, midSidewallDistance, midLoadingStationDistance);
    }   //CmdMidGearLift

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
            double xDistance, yDistance;

            switch (state)
            {
                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(State.GO_TO_MID_LIFT);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.GO_TO_MID_LIFT);
                    }
                    break;

                case GO_TO_MID_LIFT:
                    //
                    // Go towards the mid lift.
                    //
                    xDistance = 0.0;
                    yDistance = midLiftDistance;

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 2.0);
                    sm.waitForSingleEvent(event, State.VISION_GEAR_DEPLOY);
                    break;

                case VISION_GEAR_DEPLOY:
                    //
                    // Have VisionGearDeploy aligning to the peg, deploy the gear and back up.
                    //
                    if (cmdVisionDeploy.cmdPeriodic(elapsedTime))
                    {
                        sm.setState(State.GO_TO_SIDEWALL);
                    }
                    break;

                case GO_TO_SIDEWALL:
                    //
                    // Move towards the loading station side wall.
                    //
                    xDistance = robot.alliance == Robot.Alliance.RED_ALLIANCE?
                        -midSidewallDistance: midSidewallDistance;
                    yDistance = 0.0;

//                    robot.encoderXPidCtrl.setOutputRange(-1.0, 1.0);
//                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.GO_TO_LOADING_STATION);
                    break;

                case GO_TO_LOADING_STATION:
                    //
                    // Move towards the loading station. 
                    //
                    xDistance = 0.0;
                    yDistance = midLoadingStationDistance;

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    done = true;
                    sm.stop();
                    break;
            }

            robot.traceStateInfo(elapsedTime, state.toString());
        }

        return done;
    }   //cmdPeriodic

}   //class CmdMidGearLift

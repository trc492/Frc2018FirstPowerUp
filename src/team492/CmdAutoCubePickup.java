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

import team492.PixyVision.TargetInfo;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

public class CmdAutoCubePickup implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoCubePickup";

    private static enum State
    {
        START, DRIVE, PICKUP, DONE
    }

    private TrcEvent event;
    private TrcStateMachine<State> sm;
    private Robot robot;

    public CmdAutoCubePickup(Robot robot)
    {
        this.robot = robot;

        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
    }

    public void start()
    {
        stop();
        sm.start(State.START);
    }

    public void stop()
    {
        if (robot.visionPidDrive.isActive())
        {
            robot.visionPidDrive.cancel();
        }
        sm.stop();
    }

    public boolean isEnabled()
    {
        return sm.isEnabled();
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();

        if (done)
        {
        	return true;
        }

        State state = sm.getState();
        robot.dashboard.displayPrintf(1, "State: %s", state != null ? state.toString() : "Disabled");
        
        if (sm.isReady())
        {
            switch (state)
            {
                case START:
                	robot.cubePickup.deployPickup();
                    robot.cubePickup.openClaw(); // TODO: Talk to Chris about
                                                 // singular method to init Claw
                                                 // subystem
                    robot.cmdPickupCube.start();
                    double pos = RobotInfo.ELEVATOR_FLOOR_PICKUP_HEIGHT; // 6.0 inches. According to Victor, pos
                                      // parameter is in inches
                    //CodeReview: you may want to put the 6.0 inches in RobotInfo so it can be tuned.
                    //Besides, it will give it a nice name instead of a magic number.
                    robot.elevator.setPosition(pos, event, 5.0); // Need to tune
                                                                 // timeout
                                                                 // section of
                                                                 // this
                    sm.waitForSingleEvent(event, State.DRIVE);
                    break;

                case DRIVE:
                    TargetInfo target = getCubeCoords();
                    robot.visionPidDrive.setTarget(target.xDistance*1.1, target.yDistance*1.1, 0.0, false, event); // Scale by 110% to overshoot
                    //CodeReview: you may want to combine visionPidDrive.setTarget with calling Chris to grab the cube.
                    //After all, you are moving and grabbing at the same time.
                    // robot.cubePickup.setCubeTriggerEnabled(true)
                    // TODO: Talk to Chris to provide TrcDigitalTrigger to
                    // monitor proximity sensor
                    sm.waitForSingleEvent(event, State.PICKUP);
                    break;

                case PICKUP:
                    robot.cubePickup.closeClaw(); // TODO: Talk to Chris about
                                                  // singular method to grab
                                                  // cube
                    //CodeReview: Chris has the CmdPickupCube to grab the cube.
                    sm.setState(State.DONE);
                    break;

                case DONE:
                default:
                    sm.stop();
                    done = true;
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString());
        }
        return done;
    }

    private TargetInfo getCubeCoords()
    {
        TargetInfo target = robot.pixy.getTargetInfo();
        return target;
    }

}

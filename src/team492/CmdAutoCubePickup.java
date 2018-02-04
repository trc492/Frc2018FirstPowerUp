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

    /**
     * Reset the state machine and start it
     */
    public void start()
    {
        stop();
        sm.start(State.START);
    }

    /**
     * Stop the state machine. cmdPeriodic won't do anything
     */
    public void stop()
    {
        if (robot.visionPidDrive.isActive())
        {
            robot.visionPidDrive.cancel();
        }
        sm.stop();
    }

    /**
     * Is the state machine enabled?
     * @return Is the state machine enabled?
     */
    public boolean isEnabled()
    {
        return sm.isEnabled();
    }

    /**
     * Execute one cycle of the state machine and stuff.
     */
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
                	// Deploy and open cube pickup
                	robot.cubePickup.deployPickup();
                    robot.cubePickup.openClaw();
                    
                    // 6.0 inches above the ground. pos is in inches
                    double pos = RobotInfo.ELEVATOR_FLOOR_PICKUP_HEIGHT;
                    robot.elevator.setPosition(pos, event, 0.0);
                    
                    sm.waitForSingleEvent(event, State.DRIVE);
                    break;

                case DRIVE:
                    TargetInfo target = getCubeCoords();
                    robot.visionPidDrive.setTarget(target.xDistance*1.1, target.yDistance*1.1, 0.0, false, event); // Scale by 110% to overshoot
                    
                    robot.cubePickup.grabCube(0.5, event);
                    
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

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
import trclib.TrcStateMachine;

public class CmdPickupCube implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdPickupCube";

    private static enum State
    {
        GRAB_CUBE, WAIT_FOR_CUBE, DONE
    } // enum State

    private Robot robot;
    private TrcStateMachine<State> sm;

    CmdPickupCube(Robot robot)
    {
        this.robot = robot;
        sm = new TrcStateMachine<>(moduleName);
    } // CmdPickupCube

    public void start()
    {
        sm.start(State.GRAB_CUBE);
    }
    
    public void stop() {
    	robot.cubePickup.stopPickup();
    }

    //CodeReview: need a stop method.

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
                case GRAB_CUBE:
                    //
                    // Spins motors to pull in cube.
                    //
                    robot.cubePickup.grabCube(0.5);
                    sm.setState(State.WAIT_FOR_CUBE);
                    break;

                    //CodeReview: create a digital trigger and use it to signal an event.

                case WAIT_FOR_CUBE:
                    //
                    // Continues spinning motors until a cube is detected.
                    //
                    if (robot.cubePickup.cubeDetected())
                    {
                        sm.setState(State.DONE);
                    } else
                    {
                        sm.setState(State.WAIT_FOR_CUBE);
                    }
                    break;

                case DONE:
                default:
                    //
                    // Done.
                    //
                    robot.cubePickup.stopPickup();
                    sm.stop();
                    break;
            }
        }
        return done;
    }

}

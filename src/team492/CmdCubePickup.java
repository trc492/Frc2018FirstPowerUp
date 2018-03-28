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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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
import trclib.TrcRobot;
import trclib.TrcStateMachine;

public class CmdCubePickup implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoCubePickup";

    public static enum State
    {
        START, DRIVE, DONE
    }

    private Robot robot;
    private TrcEvent event, pickupEvent;
    private TrcStateMachine<State> sm;
    private double xError;

    public CmdCubePickup(Robot robot)
    {
        this.robot = robot;
        event = new TrcEvent(moduleName);
        pickupEvent = new TrcEvent(moduleName + ".cubePickup");
        sm = new TrcStateMachine<>(moduleName);
    }

    /**
     * Is the state machine enabled?
     * 
     * @return Is the state machine enabled?
     */
    public boolean isEnabled()
    {
        return sm.isEnabled();
    }

    public void start()
    {
        start(0.0);
    }

    public void start(double xError)
    {
        if (!sm.isEnabled())
        {
            this.xError = xError;
            sm.start(State.START);
        }
    }

    public void stop()
    {
        if (sm.isEnabled())
        {
            robot.cubePickup.setProximityTriggerEnabled(false, null);
            robot.pidDrive.cancel();
            sm.stop();
        }
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();

        if (!done)
        {
            State state = sm.checkReadyAndGetState();
            //
            // Print debug info.
            //
            robot.dashboard.displayPrintf(1, "State: %s", state == null? "NotReady": state);

            if (state != null)
            {
                double xDistance, yDistance;

                switch (sm.getState())
                {
                    case START:
                        // Deploy and open cube pickup
                        robot.cubePickup.deployPickup();
                        robot.cubePickup.openClaw();
                        robot.elevator.setPosition(RobotInfo.ELEVATOR_FLOOR_PICKUP_HEIGHT);
                        sm.setState(State.DRIVE);
                        break;

                    case DRIVE:
                        // Go forward the expected distance or until the cube is in possession.
                        xDistance = xError;
                        yDistance = RobotInfo.AUTO_PICKUP_CUBE_DISTANCE;
//                        yDistance = robot.frontSonarSensor != null?
//                            robot.getFrontSonarDistance(): RobotInfo.AUTO_PICKUP_CUBE_DISTANCE;
                        robot.cubePickup.grabCube(RobotInfo.PICKUP_TELEOP_POWER, pickupEvent);
//                        robot.cubePickup.setProximityTriggerEnabled(true, proximityEvent);
//                        sm.addEvent(proximityEvent);
                        sm.addEvent(pickupEvent);
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        sm.addEvent(event);
                        sm.waitForEvents(State.DONE);
                        break;

                    case DONE:
                        stop();
                        done = true;
                        break;
                }
                robot.traceStateInfo(elapsedTime, state.toString());
            }
        }

        return done;
    }

}

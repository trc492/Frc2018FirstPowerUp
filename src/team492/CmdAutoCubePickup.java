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

public class CmdAutoCubePickup implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoCubePickup";

    public static enum State
    {
        START, DRIVE, DONE
    }

    private Robot robot;
    private TrcEvent event, proximityEvent;
    private TrcStateMachine<State> sm;
//    private TrcEvent onFinishedEvent = null;
//    private double startX = 0.0, startY = 0.0;
//    private double startTime = 0.0;

    public CmdAutoCubePickup(Robot robot)
    {
        this.robot = robot;
        event = new TrcEvent(moduleName);
        proximityEvent = new TrcEvent(moduleName + ".proximity");
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

//    public double elapsedTime()
//    {
//    	return Robot.getModeElapsedTime() - startTime;
//    }
//
//    public double[] distanceMoved()
//    {
//        return new double[] { changeX(), changeY() };
//    }
//
//    private double changeX()
//    {
//    	return robot.driveBase.getXPosition() - startX;
//    }
//
//    public double changeY()
//    {
//    	return robot.driveBase.getYPosition() - startY;
//    }
//
//    /**
//     * Start this task, and signal onFinishedEvent when done
//     */
//    public void start(TrcEvent onFinishedEvent, AutoAssistStopTrigger stopTrigger)
//    {
//        stop();
//        this.onFinishedEvent = onFinishedEvent;
//        this.stopTrigger = stopTrigger;
//        startX = robot.driveBase.getXPosition();
//        startY = robot.driveBase.getYPosition();
//        startTime = Robot.getModeElapsedTime();
//        sm.start(State.START);
//    }
//
//    public void start(AutoAssistStopTrigger stopTrigger)
//    {
//        start(null, stopTrigger);
//    }
//
//    /**
//     * Start this task without signaling any event when done
//     */
//    public void start()
//    {
//        start(null, this);
//    }

//    public void start(TrcEvent onFinishedEvent)
//    {
//        start(onFinishedEvent, this);
//    }
//
//    public boolean shouldStop(double elapsedTime, double distanceX, double distanceY, boolean xStop)
//    {
//        return false;
//    }
//
    public void start()
    {
        if (!sm.isEnabled())
        {
//            onFinishedEvent = event;
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

//            if (this.stopTrigger.shouldStop(elapsedTime(), changeX(), changeY(), false))
//            {
//                stop();
//                done = true;
//            }
            if (state != null)
            {
                double xDistance, yDistance;

                switch (sm.getState())
                {
                    case START:
                        // Deploy and open cube pickup
                        robot.cubePickup.deployPickup();
                        robot.cubePickup.openClaw();
                        robot.elevator.setPosition(RobotInfo.ELEVATOR_FLOOR_PICKUP_HEIGHT, event, 0.0);
                        sm.waitForSingleEvent(event, State.DRIVE);
                        break;

                    case DRIVE:
//                      // Go forward at 60% power facing the cube
//                      robot.visionPidDrive.driveMaintainHeading(
//                          0.0, RobotInfo.AUTO_PICKUP_MOVE_POWER, 0.0);
//                      robot.cubePickup.grabCube(0.5, event);
//
//                      sm.waitForSingleEvent(event, State.PICKUP);

                        //CodeReview: change to use sonar with default distance and proximity as stopTrigger.
                        // Go forward the expected distance or until the cube is in possession.
                        xDistance = 0.0;
                        yDistance = robot.frontSonar != null?
                            robot.getFrontSonarDistance(): RobotInfo.AUTO_PICKUP_CUBE_DISTANCE;
                        robot.cubePickup.setProximityTriggerEnabled(true, proximityEvent);
                        sm.addEvent(proximityEvent);
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        sm.addEvent(event);
                        sm.waitForEvents(State.DONE);
                        break;

//                    case PICKUP:
//                        robot.visionPidDrive.cancel();
//                        robot.cubePickup.closeClaw();
//                        sm.setState(State.DONE);
//                        break;

                    case DONE:
                    default:
                        stop();
                        done = true;
//                        if(onFinishedEvent != null)
//                        {
//                            onFinishedEvent.set(true);
//                        }
                        break;
                }
                robot.traceStateInfo(elapsedTime, state.toString());
            }
        }

        return done;
    }

}

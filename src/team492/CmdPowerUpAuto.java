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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdPowerUpAuto implements TrcRobot.RobotCommand
{
    private static enum State
    {
        PICK_UP_CUBE,
        RAISE_ELEVATOR,
        DO_DELAY,
        DRIVE_FORWARD_DISTANCE,
        MOVE_SIDEWAYS,
        DRIVE_TO_TARGET,
        TURN_ROBOT,
        DEPOSIT_CUBE,
        STRAFE_TO_PLATFORM_ZONE,
        DONE
    } // enum State

    private static final String moduleName = "CmdPowerUpAuto";

    private Robot robot;
    private double delay;
    //CodeReview: alliance may be useful for others, move it to robot.java.
    private Alliance alliance;
    private String targetSide;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private int startLocation;
    private double forwardDistance;
    private int targetType;
    private CmdAutoCubePickup cmdAutoCubePickup;
    private double elevatorTargetHeight;

    CmdPowerUpAuto(Robot robot, double delay, int targetType, double forwardDistance)
    {
        DriverStation ds = DriverStation.getInstance();
        this.robot = robot;
        this.delay = delay;
        this.alliance = ds.getAlliance();
        // Gets which side is our switch and scale
        this.targetSide = ds.getGameSpecificMessage();
        this.startLocation = ds.getLocation();

        if(-1.0 == forwardDistance) 
        {
            this.forwardDistance = HalDashboard.getNumber("forwardDistance", 85.0); //TODO: change
        }                                                                           // default number
        else
        {
        	this.forwardDistance = forwardDistance;
        }
        
        this.targetType = targetType;


        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.PICK_UP_CUBE);

        robot.tracer.traceInfo(moduleName,
            "delay=%.3f, alliance=%s, targetSide=%s, startLocation=%d, forwardDist=%.1f, targetType=%d",
            "lsDist=%.1f, diagDist=%.1f", delay, alliance, targetSide, startLocation, forwardDistance, targetType);
    } // CmdPidDrive

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
        robot.dashboard.displayPrintf(1, "State: %s", state != null ? state.toString() : "Disabled");
        
        
        robot.cmdAutoCubePickup.cmdPeriodic(elapsedTime);
        
        if (sm.isReady())
        {
            state = sm.getState();
            double xDistance, yDistance;

            switch (state)
            {
                case PICK_UP_CUBE:
                    //
                    // pick up cube from floor
                    //
                	if(cmdAutoCubePickup.cmdPeriodic(elapsedTime)) {
                		sm.setState(State.RAISE_ELEVATOR);
                	}
                    break;

                case RAISE_ELEVATOR:
                	if(targetType == 1) {
                		elevatorTargetHeight = 60;
                	}
                	else {
                		elevatorTargetHeight = 27;
                	}
                	robot.elevator.setPosition(elevatorTargetHeight, event, 0.0);
                	sm.waitForSingleEvent(event, State.DO_DELAY);
                    break;

                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(State.DRIVE_FORWARD_DISTANCE);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_FORWARD_DISTANCE);
                    }
                    break;

                case DRIVE_FORWARD_DISTANCE:
                    xDistance = 0.0;
                    yDistance = forwardDistance;

                    // robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.MOVE_SIDEWAYS);
                    break;

                case MOVE_SIDEWAYS:
                    xDistance = 12;
                    yDistance = 0;
                	//TODO: Change to actual x distance
                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DRIVE_TO_TARGET:
                    xDistance = 0.0;
                    //TODO: Check numbers for yDistance expression
                    if (targetType == 0)
                    {
                    	// When the target is the switch
                    	yDistance = 168 - forwardDistance;
                    	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    	
                    }
                    else if (targetType == 1)
                    {
                        // if the target is the scale
                    	yDistance = 299.65 - forwardDistance;
                    	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    }

                    sm.waitForSingleEvent(event, State.TURN_ROBOT);
                    break;

                case TURN_ROBOT:
                	xDistance = 0;
                    yDistance = 0;
                    robot.targetHeading = 90;
                	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                	sm.waitForSingleEvent(event, State.DEPOSIT_CUBE);
                    break;

                case DEPOSIT_CUBE:
                    break;

                case STRAFE_TO_PLATFORM_ZONE:
                	xDistance = 30;
                    yDistance = 0;
                	//TODO: Get actual xDistance
                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
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
    } // cmdPeriodic

} // class CmdPowerUpAuto

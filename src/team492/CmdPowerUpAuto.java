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

@SuppressWarnings("unused")
class CmdPowerUpAuto implements TrcRobot.RobotCommand
{
    private static enum State
    {
        DO_DELAY,
        DRIVE_FORWARD_DISTANCE,
        TURN_TO_SWITCH,
        MOVE_SIDEWAYS,
        SWITCH_TURN,
        DRIVE_TO_TARGET,
        STRAFE_TO_SWITCH,
        FLIP_CUBE,
        STRAFE_FROM_SWITCH,
        DRIVE_TO_SECOND_CUBE,
        TURN_AROUND,
        START_STRAFE,
        STRAFE_TO_SECOND_CUBE,
        START_SECOND_PICKUP,
        PICKUP_SECOND_CUBE,
        BACKUP_WITH_SECOND_CUBE,
        REPOSITION_TURN,
        DRIVE_TO_SECOND_TARGET,
        CLOSE_TARGET_DRIVE,
        FAR_TARGET_DRIVE,
        TURN_ROBOT,
        ADVANCE_TO_SCALE,
        TURN_AGAIN,
        RAISE_ELEVATOR,
        APPROACH_FINAL_TARGET,
        DEPOSIT_CUBE,
        LOWER_ELEVATOR,
        DONE
    } // enum State

    private static final String moduleName = "CmdPowerUpAuto";

    private Robot robot;
    private double delay;
    private double forwardDistance;
    private boolean sideApproach;
    private double startPosition;

    private String targetSide;
    private int startLocation;
    private boolean rightSwitch;
    private boolean rightScale;
    private double targetLocation;

    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    private boolean rightTurn;
    private double cubeStrafeDistance;
    private double cubeAcquireDistance;
    private boolean pickupCancelled;

    CmdPowerUpAuto(Robot robot, double delay, double forwardDistance, boolean sideApproach, double startPosition)
    {
        this.robot = robot;
        this.delay = delay;
        if(-1.0 == forwardDistance) 
        {
            this.forwardDistance = HalDashboard.getNumber("Forward Distance", 10.0); 
        }                                                                           
        else
        {
            this.forwardDistance = forwardDistance;
        }
        this.sideApproach = sideApproach;
        this.startPosition = startPosition;

        this.targetSide = robot.ds.getGameSpecificMessage();
        this.startLocation = robot.ds.getLocation();
        this.rightSwitch = (targetSide.charAt(0) == 'R');
        this.rightScale = (targetSide.charAt(1) == 'R');

        if(rightSwitch)
        {
        	this.targetLocation = RobotInfo.RIGHT_SWITCH_LOCATION;
        }
        else
        {
        	this.targetLocation = RobotInfo.LEFT_SWITCH_LOCATION;
        }

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);

        robot.tracer.traceInfo(moduleName,
            "delay=%.3f, alliance=%s, targetSide=%s, startLocation=%d, forwardDist=%.1f, startPosition=%.1f",
             delay, robot.alliance, targetSide, startLocation, forwardDistance, startPosition);
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

        if (sm.isReady())
        {
            state = sm.getState();
            double xDistance, yDistance;
            State nextState;

            switch (state)
            {
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
                    if(targetLocation == startPosition)
                    {
                        //
                        // Don't need to go across the other side.
                        //
                        yDistance = RobotInfo.AUTO_DISTANCE_TO_SWITCH;
                        nextState = State.STRAFE_TO_SWITCH;
                    }
                    else
                    {
                        //
                        // Go forward, turn and cross to the other side.
                        //
                        yDistance = forwardDistance;
                        nextState = State.TURN_TO_SWITCH;
                    }
                    robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    rightTurn = rightSwitch;
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case TURN_TO_SWITCH:
                    //
                    // Crossing to the other side, determine which way to turn
                    //
                    xDistance = 0.0;
                    yDistance = 0.0;
                    if(rightTurn)
                    {
                        robot.targetHeading = 90;
                    }
                    else
                    {
                        robot.targetHeading = -90;
                    }
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.MOVE_SIDEWAYS);
                    break;

                case MOVE_SIDEWAYS:
                    xDistance = 0.0;
                    yDistance = Math.abs(targetLocation - startPosition);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.SWITCH_TURN);
                    break;

                case SWITCH_TURN:
                    xDistance = 0;
                    yDistance = 0;
                    robot.targetHeading = 0.0;
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_TARGET);
                    break;

                case DRIVE_TO_TARGET:
                    xDistance = 0.0;
                    yDistance = RobotInfo.AUTO_DISTANCE_TO_SWITCH - forwardDistance;
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.STRAFE_TO_SWITCH);
                    break;

                case STRAFE_TO_SWITCH:
                    if(rightSwitch)
                    {
                        xDistance = -(RobotInfo.SWITCH_STRAFE_DISTANCE + 2.0);
                    }
                    else
                    {
                        xDistance = RobotInfo.SWITCH_STRAFE_DISTANCE + 2.0;
                    }
                    yDistance = 0.0;
                    robot.encoderXPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.FLIP_CUBE);
                    break;

                case FLIP_CUBE:
                    if(rightSwitch)
                    {
                        robot.leftFlipper.extend();
                    }
                    else
                    {
                        robot.rightFlipper.extend();
                    }
                    timer.set(0.3, event);
                    sm.waitForSingleEvent(event, State.STRAFE_FROM_SWITCH);
                    break;

                case STRAFE_FROM_SWITCH:
                    if(rightSwitch)
                    {
                        robot.leftFlipper.retract();
                        xDistance = RobotInfo.SWITCH_STRAFE_DISTANCE;
                    }
                    else
                    {
                        robot.rightFlipper.retract();
                        xDistance = -RobotInfo.SWITCH_STRAFE_DISTANCE;
                    }
                    yDistance = 0.0;
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_SECOND_CUBE);
                    break;

                case DRIVE_TO_SECOND_CUBE:
                    xDistance = 0.0;
                    yDistance = RobotInfo.ADVANCE_TO_SECOND_CUBE_DISTANCE;
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_AROUND);
                    break;

                case TURN_AROUND:
                    xDistance = 0.0;
                    yDistance = 0.0;
                    robot.targetHeading = 180.0;
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.START_STRAFE);
                    break;

                case START_STRAFE:
                    boolean strafeRight = rightSwitch;
                    robot.cmdStrafeUntilCube.start(strafeRight, this::shouldXStop);
                    sm.setState(State.STRAFE_TO_SECOND_CUBE);
                    break;

                case STRAFE_TO_SECOND_CUBE:
                    if(robot.cmdStrafeUntilCube.cmdPeriodic(elapsedTime))
                    {
                        sm.setState(State.START_SECOND_PICKUP);
                    }
                    cubeStrafeDistance = robot.cmdStrafeUntilCube.changeX();
                    break;

                case START_SECOND_PICKUP:
                    robot.cmdAutoCubePickup.start(this::shouldYStop);
                    sm.setState(State.PICKUP_SECOND_CUBE);
                    break;

                case PICKUP_SECOND_CUBE:
                    if(robot.cmdAutoCubePickup.cmdPeriodic(elapsedTime))
                    {
                    	if(pickupCancelled)
                    	{
                    		// might want to make this do something else if a cube is not found
                    		sm.setState(State.DONE);
                    	}
                    	else
                    	{
                            sm.setState(State.BACKUP_WITH_SECOND_CUBE);
                    	}
                    }
                    cubeAcquireDistance = robot.cmdAutoCubePickup.changeY();
                    break;

                case BACKUP_WITH_SECOND_CUBE:
                    xDistance = 0;
                    yDistance = -cubeAcquireDistance;
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.REPOSITION_TURN);
                    break;

                case REPOSITION_TURN:
                    xDistance = 0;
                    yDistance = 0;
                    if(rightScale)
                    {
                        robot.targetHeading = 90;   //Head east
                    }
                    else
                    {
                        robot.targetHeading = -90;  //Head west
                    }
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_SECOND_TARGET);
                    break;

                case DRIVE_TO_SECOND_TARGET:
            		xDistance = 0;
                	if(rightScale==rightSwitch)
                	{
                		yDistance = RobotInfo.SCALE_FRONT_POSITION - (RobotInfo.START_POS_3 - cubeStrafeDistance);
                    }
                    else
                    {
                    	yDistance = RobotInfo.SCALE_FRONT_POSITION + (RobotInfo.START_POS_3 - cubeStrafeDistance);
                    }
                	
                	if(sideApproach)
                	{
                		yDistance += RobotInfo.SCALE_SIDE_POSITION - RobotInfo.SCALE_FRONT_POSITION;
                	}
                	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                	sm.waitForSingleEvent(event, State.TURN_ROBOT);
                    break;

                case TURN_ROBOT:
                    xDistance = yDistance = 0.0;
                    robot.targetHeading = 0.0;  //turning back to north
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    if(sideApproach)
                    {
                        sm.waitForSingleEvent(event, State.ADVANCE_TO_SCALE);
                    }
                    else
                    {
                        sm.waitForSingleEvent(event, State.RAISE_ELEVATOR);
                    }
                    break;
                	
                case ADVANCE_TO_SCALE:
                	xDistance = 0.0;
                    yDistance = RobotInfo.ADVANCE_AROUND_SCALE_DISTANCE;
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_AGAIN);
                	break;
                	
                case TURN_AGAIN:
                    xDistance = yDistance = 0.0;
                    robot.targetHeading = rightScale? 270.0: 90.0;
                	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                	sm.waitForSingleEvent(event, State.RAISE_ELEVATOR);
                	break;
                	
                case RAISE_ELEVATOR:
                	robot.elevator.setPosition(RobotInfo.ELEVATOR_MAX_HEIGHT, event, 0.0);
                	sm.waitForSingleEvent(event, State.APPROACH_FINAL_TARGET);
                    break;
                    
                case APPROACH_FINAL_TARGET:
                	xDistance = 0.0;
                	if(sideApproach)
                	{
                        yDistance = RobotInfo.FINAL_SIDE_SCALE_APPROACH_DISTANCE;
                	}
                	else
                	{
                        yDistance = RobotInfo.FINAL_FRONT_SCALE_APPROACH_DISTANCE;
                	}
                	robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DEPOSIT_CUBE);
                	break;

                case DEPOSIT_CUBE:
                	robot.cubePickup.dropCube(0.5);
                	timer.set(0.3, event);
                    sm.waitForSingleEvent(event, State.LOWER_ELEVATOR);
                    break;
                    
                case LOWER_ELEVATOR:
                	robot.elevator.setPosition(RobotInfo.ELEVATOR_MIN_HEIGHT, event, 0.0);
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


    public boolean shouldXStop(double elapsedTime, double changeX, double changeY)
    {
        return Math.abs(changeX) > RobotInfo.STRAFE_TO_SECOND_CUBE_DISTANCE;
    }
    

    public boolean shouldYStop(double elapsedTime, double changeX, double changeY)
    {
    	pickupCancelled = changeY > RobotInfo.MAX_CUBE_DISTANCE;
        return pickupCancelled;
    }

} // class CmdPowerUpAuto

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
        APPROACH_SCALE_SIDE,
        APPROACH_SCALE_FRONT,
        TURN_ROBOT,
        APPROACH_TARGET,
        DEPOSIT_CUBE,
        BACKUP,
        AFTER_ACTION_TURN,
        PICKUP_SECOND_CUBE,
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
    private boolean sideApproach;
    private int afterAction;
    private boolean rightTurn;
    private boolean lastTurn;
    private boolean rightSwitch;
    private boolean rightScale;
    private double startPosition;
    private double targetLocation;
    //TODO: implement all of these variables
    
    //TODO: add these constants to RobotInfo and fix them
    private static final double AUTO_DISTANCE_TO_SWITCH = 110.0;
    private static final double AUTO_DISTANCE_TO_SCALE = 208.0;
    private static final double SCALE_SIDE_APPROACH_DISTANCE = 101.0;
    private static final double SCALE_FRONT_APPROACH_DISTANCE = 71.0;
    private static final double FINAL_FRONT_SCALE_APPROACH_DISTANCE = 62.0;
    private static final double FINAL_SIDE_SCALE_APPROACH_DISTANCE = 21.0;
    private static final double RIGHT_SWITCH_LOCATION = 59.0;
    private static final double LEFT_SWITCH_LOCATION = -59.0;
    private static final double RIGHT_SCALE_LOCATION = 126.0;
    private static final double LEFT_SCALE_LOCATION = -126.0;
    private static final double SCALE_TARGET_HEIGHT = 60.0;
    private static final double SWITCH_TARGET_HEIGHT = 27.0;
    private static final double FRONT_SCALE_BACKUP_DISTANCE = 4200000;
    private static final double SIDE_SCALE_BACKUP_DISTANCE = 4200000;

    CmdPowerUpAuto(Robot robot, double delay, int targetType, double forwardDistance, boolean sideApproach, int afterAction, double startPosition)
    {
        DriverStation ds = DriverStation.getInstance();
        this.robot = robot;
        this.delay = delay;
        this.alliance = ds.getAlliance();
        // Gets which side is our switch and scale
        this.targetSide = ds.getGameSpecificMessage();
        this.rightSwitch = (targetSide.charAt(0) == 'R');
        this.rightScale = (targetSide.charAt(1) == 'R');
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
        this.sideApproach = sideApproach;
        this.afterAction = afterAction;
        this.startPosition = startPosition;
        if(targetType == 1)
        {
        	if(rightScale)
        	{
        		this.targetLocation = RIGHT_SCALE_LOCATION;
        	}
        	else
        	{
        		this.targetLocation = LEFT_SCALE_LOCATION;
        	}
        }
        else
        {
        	if(rightSwitch)
        	{
        		this.targetLocation = RIGHT_SWITCH_LOCATION;
        	}
        	else
        	{
        		this.targetLocation = LEFT_SWITCH_LOCATION;
        	}
        }
        if(targetType == 1)
        {
    		elevatorTargetHeight = SCALE_TARGET_HEIGHT;
    	}
    	else 
        {
    		elevatorTargetHeight = SWITCH_TARGET_HEIGHT;
    	}

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
                	//TODO: dont raise elevator yet
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

                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.MOVE_SIDEWAYS);
                    break;

                case MOVE_SIDEWAYS:
                    xDistance = targetLocation - startPosition;
                    yDistance = 0;
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_TARGET);
                    break;

                case DRIVE_TO_TARGET:
                    xDistance = 0.0;
                    if (targetType == 0)
                    {
                    	// When the target is the switch
                    	yDistance = AUTO_DISTANCE_TO_SWITCH - forwardDistance;
                    	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                		sm.waitForSingleEvent(event, State.DEPOSIT_CUBE);
                    }
                    else
                    {
                        // if the target is the scale
                    	yDistance = AUTO_DISTANCE_TO_SCALE - forwardDistance;
                    	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    	if(sideApproach)
                	    {
                            sm.waitForSingleEvent(event, State.APPROACH_SCALE_SIDE);
                	    }
                	    else
                	    {
                		    sm.waitForSingleEvent(event, State.APPROACH_SCALE_FRONT);
                	    }
                    }
                    break;
                    
                case APPROACH_SCALE_SIDE:
                    yDistance = SCALE_SIDE_APPROACH_DISTANCE;
                	xDistance = 0;
                	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                	sm.waitForSingleEvent(event, State.TURN_ROBOT);
                	if(rightScale)
                	{
                		rightTurn = false;
                	}
                	else
                	{
                		rightTurn = true;
                	}
                    break;
                    
                case APPROACH_SCALE_FRONT:
                	yDistance = 0;
                	if(rightScale)
                	{
                    	xDistance = SCALE_FRONT_APPROACH_DISTANCE;
                	}
                	else
                	{
                    	xDistance = -SCALE_FRONT_APPROACH_DISTANCE;
                	}
                	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                	sm.waitForSingleEvent(event, State.APPROACH_TARGET);
                	break;

                case TURN_ROBOT:
                	xDistance = 0;
                    yDistance = 0;
                    if(rightTurn)
                    {
                        robot.targetHeading += 90;
                    }
                    else
                    {
                    	robot.targetHeading -= 90;
                    }
                	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.APPROACH_TARGET);
                    break;
                    
                case APPROACH_TARGET:
                	xDistance = 0.0;
                	// TODO: change yDistance based on approach if necessary
                	if(sideApproach)
                	{
                        yDistance = FINAL_SIDE_SCALE_APPROACH_DISTANCE;
                	}
                	else
                	{
                        yDistance = FINAL_FRONT_SCALE_APPROACH_DISTANCE;
                	}

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DEPOSIT_CUBE);
                	break;

                case DEPOSIT_CUBE:
                	robot.cubePickup.grabCube(-0.5, event);
                	if(0 == afterAction)
                	{
                    	sm.waitForSingleEvent(event, State.DONE);
                	}
                	else
                	{
                    	sm.waitForSingleEvent(event, State.PICKUP_SECOND_CUBE);
                	}
                    break;
                    
                case BACKUP:
                	xDistance = 0;
                	if(sideApproach)
                	{
                    	yDistance = FRONT_SCALE_BACKUP_DISTANCE;
                	}
                	else
                	{
                    	yDistance = FRONT_SCALE_BACKUP_DISTANCE;
                	}
                	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                	sm.waitForSingleEvent(event, State.APPROACH_TARGET);
                	break;
                	
                case AFTER_ACTION_TURN:
                	xDistance = 0;
                    yDistance = 0;
                    if(rightTurn)
                    {
                        robot.targetHeading += 90;
                    }
                    else
                    {
                    	robot.targetHeading -= 90;
                    }
                	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    break;
                	
                case PICKUP_SECOND_CUBE:
                	//TODO: make this do something
                	if(afterAction == 1)
                	{
                		
                	}
                	else
                	{
                		
                	}
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

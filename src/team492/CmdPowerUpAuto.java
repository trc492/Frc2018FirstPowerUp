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
        MOVE_SIDEWAYS,
        DRIVE_TO_TARGET,
        FLIP_CUBE,
        APPROACH_SCALE_SIDE,
        APPROACH_SCALE_FRONT,
        TURN_ROBOT,
        APPROACH_TARGET,
        DEPOSIT_CUBE,
        DRIVE_TO_SECOND_CUBE,
        TURN_AROUND,
        STRAFE_TO_SECOND_CUBE,
        START_SECOND_PICKUP,
        PICKUP_SECOND_CUBE,
        BACKUP_WITH_SECOND_CUBE,
        REPOSITION_WITH_SECOND_CUBE,
        REPOSITION_TURN,
        RAISE_ELEVATOR,
        DONE
    } // enum State

    private static final String moduleName = "CmdPowerUpAuto";

    private Robot robot;
    private double delay;
    //CodeReview: alliance may be useful for others, move it to robot.java.
    private String targetSide;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private int startLocation;
    private double forwardDistance;
    private boolean sideApproach;
    private boolean rightTurn;
    private boolean rightSwitch;
    private boolean rightScale;
    private double startPosition;
    private double targetLocation;
    

    CmdPowerUpAuto(Robot robot, double delay, double forwardDistance, boolean sideApproach, double startPosition)
    {
        DriverStation ds = DriverStation.getInstance();
        this.robot = robot;
        this.delay = delay;
        this.targetSide = ds.getGameSpecificMessage();
        this.rightSwitch = (targetSide.charAt(0) == 'R');
        this.rightScale = (targetSide.charAt(1) == 'R');
        this.startLocation = ds.getLocation();

        if(-1.0 == forwardDistance) 
        {
            this.forwardDistance = HalDashboard.getNumber("Forward Distance", 4.0); 
        }                                                                           
        else
        {
        	this.forwardDistance = forwardDistance;
        }
        this.sideApproach = sideApproach;
        this.startPosition = startPosition;
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
            "lsDist=%.1f, diagDist=%.1f", delay, robot.alliance, targetSide, startLocation, forwardDistance, startPosition);
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
        
        
//        if(robot.cmdAutoCubePickup.isEnabled())
//        {
//        	robot.cmdAutoCubePickup.cmdPeriodic(elapsedTime);
//        }
//        
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
                    
                //if statement -- do you need to go to the other side?
                //Instead of moving sideways --- turn then forwards
                case MOVE_SIDEWAYS:
                    xDistance = targetLocation - startPosition;
                    yDistance = 0;
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_TARGET);
                    break;

                case DRIVE_TO_TARGET:
                    xDistance = 0.0;
                    yDistance = RobotInfo.AUTO_DISTANCE_TO_SWITCH - forwardDistance;
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
                	sm.setState(State.DRIVE_TO_SECOND_CUBE);
                	break;
            	
            	//300 ms delay before next state
                //retract the flipper
                case DRIVE_TO_SECOND_CUBE:
                	xDistance = 0.0;
                    yDistance = RobotInfo.ADVANCE_TO_SECOND_CUBE_DISTANCE;
                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_AROUND);
                	break;
                	
                case TURN_AROUND:
                	xDistance = 0;
                    yDistance = 0;
                    robot.targetHeading += 180;
                	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                	sm.waitForSingleEvent(event, State.STRAFE_TO_SECOND_CUBE);
                    break;
                    
                //Abhay has a method!!!
                case STRAFE_TO_SECOND_CUBE:
                    yDistance = 0;
                    if(rightSwitch)
                    {
                        xDistance = RobotInfo.STRAFE_TO_SECOND_CUBE_DISTANCE;
                    }
                    else
                    {
                        xDistance = -RobotInfo.STRAFE_TO_SECOND_CUBE_DISTANCE;
                    }
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.START_SECOND_PICKUP);
                    break;
                    
                case START_SECOND_PICKUP:
                    robot.cmdAutoCubePickup.start();
                    sm.setState(State.PICKUP_SECOND_CUBE);
                    break;

                case PICKUP_SECOND_CUBE:
                    if(robot.cmdAutoCubePickup.cmdPeriodic(elapsedTime))
                    {
                        sm.setState(State.BACKUP_WITH_SECOND_CUBE);
                    }
                	break;
                	
                case BACKUP_WITH_SECOND_CUBE:
                	xDistance = 0;
                    yDistance = -RobotInfo.SECOND_CUBE_BACKUP_DISTANCE;
                	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.REPOSITION_WITH_SECOND_CUBE);
                	break;
                		 	
                case REPOSITION_WITH_SECOND_CUBE:
                	if(rightScale)
                	{
                    	xDistance = -RobotInfo.ADVANCE_TO_SECOND_CUBE_DISTANCE;
                	}
                	else
                	{
                    	xDistance = RobotInfo.ADVANCE_TO_SECOND_CUBE_DISTANCE;
                	}
                    yDistance = 0.0;

                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.REPOSITION_TURN);
                	break;
                	
                case REPOSITION_TURN:
                	xDistance = 0;
                    yDistance = 0;
                    robot.targetHeading += 180;
                	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                	if(sideApproach)
            	    {
                        sm.waitForSingleEvent(event, State.APPROACH_SCALE_SIDE);
            	    }
            	    else
            	    {
            		    sm.waitForSingleEvent(event, State.APPROACH_SCALE_FRONT);
            	    }
                	break;
                	
                case APPROACH_SCALE_SIDE:
                    yDistance = RobotInfo.SCALE_SIDE_APPROACH_DISTANCE;
                	xDistance = 0;
                	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                	sm.waitForSingleEvent(event, State.TURN_ROBOT);
                	rightTurn = !rightScale;
                    break;
                    
                case APPROACH_SCALE_FRONT:
                	yDistance = 0;
                	if(rightScale)
                	{
                    	xDistance = -RobotInfo.SCALE_FRONT_APPROACH_DISTANCE;
                	}
                	else
                	{
                    	xDistance = RobotInfo.SCALE_FRONT_APPROACH_DISTANCE;
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
                	if(sideApproach)
                	{
                        yDistance = RobotInfo.FINAL_SIDE_SCALE_APPROACH_DISTANCE;
                	}
                	else
                	{
                        yDistance = RobotInfo.FINAL_FRONT_SCALE_APPROACH_DISTANCE;
                	}

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.RAISE_ELEVATOR);
                	break;
                	
                case RAISE_ELEVATOR:
                    //TODO: start raising elevator earlier
                	robot.elevator.setPosition(RobotInfo.SCALE_TARGET_HEIGHT, event, 0.0);
                	sm.waitForSingleEvent(event, State.DEPOSIT_CUBE);
                    break;

                case DEPOSIT_CUBE:
                	robot.cubePickup.grabCube(-0.5, event);
                	//TODO: Check if this actually works 
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

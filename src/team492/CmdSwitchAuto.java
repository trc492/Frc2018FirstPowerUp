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

import hallib.HalDashboard;
import trclib.TrcAnalogInput;
import trclib.TrcAnalogTrigger;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdSwitchAuto implements TrcRobot.RobotCommand
{
    private static enum State
    {
        DO_DELAY,
        DRIVE_FORWARD_DISTANCE,
        TURN_TO_SWITCH,
        MOVE_ACROSS,
        //SWITCH_TURN,
        //DRIVE_TO_TARGET,
        CHECK_SONAR_DISTANCE,
        SONAR_STRAFE_TO_SWITCH,
        STRAFE_TO_SWITCH,
        SONAR_FLIP_CUBE,
        FLIP_CUBE,
        STRAFE_FROM_SWITCH,
        DRIVE_PAST_SWITCH,
        TURN_SOUTH,
        POSITION_TO_STRAFE,
        SONAR_POSITION_TO_STRAFE,
        START_STRAFE,
        STRAFE_TO_SECOND_CUBE,
        PRECISION_STRAFE,
        START_SECOND_PICKUP,
        PICKUP_SECOND_CUBE,
        BACKUP_WITH_SECOND_CUBE,
        LIFT_CUBE_SLIGHTLY,
        DONE
    } // enum State

    private static final String moduleName = "CmdSwitchAuto";
    private static final double[] sonarTriggerPoints = {8.0, 32.0};
    
    // TODO: move these to RobotInfo
    private static final double OPPOSITE_SWITCH_OVERSHOOT = 21.0;
    private static final double STRAFE_FROM_SWITCH_DISTANCE = 12.0;
    private static final double POSITION_TO_STRAFE_DISTANCE = 62.0;

    private Robot robot;
    private double delay;
    private double forwardDistance;
    private double startPosition;
    private boolean flipInFlight;

    private String targetSide;
    private int startLocation;
    private boolean rightSwitch;
    private double targetLocation;

    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private TrcAnalogTrigger<TrcAnalogInput.DataType> sonarTrigger = null;
    private TrcEvent sonarEvent;
    private double xPowerLimit, yPowerLimit;

    private double xStart, yStart;
    private double cubeStrafeDistance;
    private double lastXPosition;
    private Double visionTarget;
    private double sonarDistance;

    CmdSwitchAuto(Robot robot, double delay, double forwardDistance, double startPosition, 
    		boolean flipInFlight)
    {
        this.robot = robot;
        this.delay = delay;
        // if forwardDistance is -1, it means the driver picked "custom".
        this.forwardDistance = forwardDistance != -1.0?
            forwardDistance: HalDashboard.getNumber("Auto/Forward Distance", 10.0);
        this.startPosition = startPosition;
        this.flipInFlight = flipInFlight;

        this.targetSide = robot.ds.getGameSpecificMessage();
        this.startLocation = robot.ds.getLocation();
        this.rightSwitch = (targetSide.charAt(0) == 'R');
        this.targetLocation = rightSwitch? RobotInfo.RIGHT_SWITCH_LOCATION: RobotInfo.LEFT_SWITCH_LOCATION;

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);

        sonarTrigger = new TrcAnalogTrigger<>(
            "SonarTrigger", rightSwitch? robot.rightSonarSensor: robot.leftSonarSensor,
            0, TrcAnalogInput.DataType.INPUT_DATA, sonarTriggerPoints, this::sonarTriggerEvent);
        sonarEvent = new TrcEvent("SonarEvent");

        xPowerLimit = robot.encoderXPidCtrl.getOutputLimit();
        yPowerLimit = robot.encoderYPidCtrl.getOutputLimit();

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
        final String funcName = "PowerUpAutoPeriodic";

        boolean done = !sm.isEnabled();

        if (!done)
        {
            State state = sm.checkReadyAndGetState();
            //
            // Print debug info.
            //
            robot.dashboard.displayPrintf(1, "State: %s", state != null? state: sm.getState());

            if (state != null)
            {
                double xDistance, yDistance;
                State nextState;
                boolean traceState = true;

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
                        if (targetLocation == startPosition)
                        {
                            //
                            // Don't need to go across the other side.
                            //
                            if (rightSwitch)
                            {
                                robot.rightSonarArray.startRanging(true);
                            }
                            else
                            {
                                robot.leftSonarArray.startRanging(true);
                            }

                            yDistance = RobotInfo.AUTO_DISTANCE_TO_SWITCH;
                            if (flipInFlight)
                            {
//                              yPowerLimit = robot.encoderYPidCtrl.getOutputLimit();
//                              robot.encoderYPidCtrl.setOutputLimit(0.5);
                                // Need to slow down sooner so the cube doesn't fly over the switch.
                                sonarTrigger.setTaskEnabled(true);
                                sm.addEvent(sonarEvent);
                                yDistance -= 16;
                            }
                            nextState = State.CHECK_SONAR_DISTANCE;
                        }
                        else
                        {
                            //
                            // TODO: if crossing to the other side, you may want to have an option
                            // not to do it in case your alliance partner can do it. It will save you time
                            // to focus on doing the scale.
                            //
                            // Go forward, turn and cross to the other side.
                            //
                            yDistance = forwardDistance;
                            nextState = State.TURN_TO_SWITCH;
                        }
                        robot.cubePickup.deployPickup();
                        // We are actually moving backward because we start by parking backwards,
                        // so make yDistance negative.
                        robot.pidDrive.setTarget(xDistance, -yDistance, robot.targetHeading, false, event);
                        sm.addEvent(event);
                        sm.waitForEvents(nextState);
                        break;

                    case TURN_TO_SWITCH:
                        //
                        // Crossing to the other side, determine which way to turn
                        //
                        xDistance = yDistance = 0.0;
                        // CodeReview: Should we go across backward? If so, need to switch the heading.
                        robot.targetHeading = rightSwitch? RobotInfo.DRIVE_HEADING_EAST: RobotInfo.DRIVE_HEADING_WEST;
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, State.MOVE_ACROSS);
                        break;

                    case MOVE_ACROSS:
                        xDistance = 0.0;
                        yDistance = Math.abs(targetLocation - startPosition) - OPPOSITE_SWITCH_OVERSHOOT;
                        // CodeReview: If going across backward, need to make yDistance negative.
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, State.STRAFE_TO_SWITCH);
                        break;

//                    case SWITCH_TURN:
//                        xDistance = yDistance = 0.0;
//                        robot.targetHeading = RobotInfo.DRIVE_HEADING_SOUTH;
//                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
//                        sm.waitForSingleEvent(event, State.DRIVE_TO_TARGET);
//                        break;

//                    case DRIVE_TO_TARGET:
//                        if (rightSwitch)
//                        {
//                            robot.rightSonarArray.startRanging(true);
//                        }
//                        else
//                        {
//                            robot.leftSonarArray.startRanging(true);
//                        }
//
//                        xDistance = 0.0;
//                        yDistance = RobotInfo.AUTO_DISTANCE_TO_SWITCH - forwardDistance;
//                        if (flipInFlight)
//                        {
//                            sonarTrigger.setTaskEnabled(true);
//                            sm.addEvent(sonarEvent);
//                            yDistance -= 10;
////                            yDistance += RobotInfo.ADVANCE_TO_SECOND_CUBE_DISTANCE;
//                        }
//
//                        // We are going backward, so make yDistance negative.
//                        robot.pidDrive.setTarget(xDistance, -yDistance, robot.targetHeading, false, event);
//                        sm.addEvent(event);
//                        sm.waitForEvents(State.CHECK_SONAR_DISTANCE);
//                        break;

                    case CHECK_SONAR_DISTANCE:
                        sonarDistance = rightSwitch? robot.getRightSonarDistance(): robot.getLeftSonarDistance();
                        // Since we moved backward, yPos is negative, let's make yStart positive.
                        yStart = -robot.driveBase.getYPosition();
                        robot.tracer.traceInfo(funcName, "sonarDistance=%.1f, yPos=%.1f", sonarDistance, yStart);

                        if (flipInFlight)
                        {
                            sonarTrigger.setTaskEnabled(false);
//                            robot.encoderYPidCtrl.setOutputLimit(yPowerLimit);
                        }

                        if (sonarDistance < RobotInfo.SWITCH_SONAR_DISTANCE_THRESHOLD)
                        {
                            sm.setState(State.SONAR_FLIP_CUBE);
                        }
                        else if (flipInFlight)
                        {
                            // We are beyond legal distance to throw the cube, go to the AUTO_DISTANCE_TO_SWITCH
                            // and strafe to within legal distance.
                            xDistance = 0.0;
                            yDistance = RobotInfo.AUTO_DISTANCE_TO_SWITCH - robot.driveBase.getYPosition();
                            robot.pidDrive.setTarget(xDistance, -yDistance, robot.targetHeading, false, event);
                            sm.waitForSingleEvent(event, State.SONAR_STRAFE_TO_SWITCH);
                        }
                        else
                        {
                            sm.setState(State.SONAR_STRAFE_TO_SWITCH);
                        }
                        break;

                    case SONAR_STRAFE_TO_SWITCH:
                        // Since we moved backward, yPos is negative, let's make yStart positive.
                        yStart = -robot.driveBase.getYPosition();
                        // Add a few more inches to make sure we are within rule.
                        xDistance = sonarDistance - RobotInfo.SWITCH_SONAR_DISTANCE_THRESHOLD + 3.0;
                        if (!rightSwitch) xDistance = -xDistance;
                        yDistance = 0.0;
                        xPowerLimit = robot.encoderXPidCtrl.getOutputLimit();
                        robot.encoderXPidCtrl.setOutputLimit(0.5);
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, State.SONAR_FLIP_CUBE);
                        break;
                        
                    case STRAFE_TO_SWITCH:
                    	//TODO: strafe distance probably needs to be tuned down a lot
                    	yDistance = 0.0;
                    	xDistance = RobotInfo.AUTO_DISTANCE_TO_SWITCH - forwardDistance;
                    	if(rightSwitch) xDistance = -xDistance;
                    	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    	sm.waitForSingleEvent(event, State.FLIP_CUBE);
                    	break;

                    case SONAR_FLIP_CUBE:
                        // Restoring xPowerLimit if it has changed.
                        robot.encoderXPidCtrl.setOutputLimit(xPowerLimit);
                        if(rightSwitch)
                        {
                            robot.rightFlipper.extend();
                            robot.rightSonarArray.stopRanging();
                        }
                        else
                        {
                            robot.leftFlipper.extend();
                            robot.leftSonarArray.stopRanging();
                        }
                        // CodeReview: Can we not wait???
                        timer.set(0.5, event);
                        sm.waitForSingleEvent(event, State.SONAR_POSITION_TO_STRAFE);
                        break;
                        
                    case FLIP_CUBE:
                    	if((rightSwitch && forwardDistance == RobotInfo.FWD_DISTANCE_3) 
                    			|| (!rightSwitch && forwardDistance != RobotInfo.FWD_DISTANCE_3))
                    	{
                    		robot.rightFlipper.extend();
                    	}
                    	else
                    	{
                    		robot.leftFlipper.extend();
                    	}
                    	timer.set(0.3, event);
                    	if(forwardDistance == RobotInfo.FWD_DISTANCE_3)
                    	{
                    		sm.waitForSingleEvent(event, State.STRAFE_FROM_SWITCH);
                    	}
                    	sm.waitForSingleEvent(event, State.TURN_SOUTH);
                    	break;
                    	
                    case STRAFE_FROM_SWITCH:
                    	yDistance = 0.0;
                    	xDistance = STRAFE_FROM_SWITCH_DISTANCE;
                    	if(!rightSwitch) xDistance = -xDistance;
                    	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    	sm.waitForSingleEvent(event, State.TURN_SOUTH);
                    	break;
                    	
                    case DRIVE_PAST_SWITCH:
                    	xDistance = 0.0;
                    	yDistance = OPPOSITE_SWITCH_OVERSHOOT;
                    	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    	sm.waitForSingleEvent(event, State.TURN_SOUTH);
                    	break;
                    	
                    case TURN_SOUTH:
                    	xDistance = yDistance = 0.0;
                    	robot.targetHeading = RobotInfo.DRIVE_HEADING_SOUTH;
                    	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    	if(forwardDistance == RobotInfo.FWD_DISTANCE_3)
                    	{
                    	sm.waitForSingleEvent(event, State.PRECISION_STRAFE);
                    	}
                    	else
                    	{
                    		sm.waitForSingleEvent(event, State.POSITION_TO_STRAFE);
                    	}
                    	break;
                    	
                    case POSITION_TO_STRAFE:
                    	xDistance = 0.0;
                    	yDistance = POSITION_TO_STRAFE_DISTANCE;
                    	robot.pidDrive.setTarget(xDistance, -yDistance, robot.targetHeading, false, event);
                    	sm.waitForSingleEvent(event, State.START_STRAFE);
                    	break;

                    case SONAR_POSITION_TO_STRAFE:
                        xDistance = 0.0;
//                        yDistance = RobotInfo.ADVANCE_TO_SECOND_CUBE_DISTANCE;
                        yDistance = RobotInfo.AUTO_DISTANCE_TO_SWITCH + RobotInfo.ADVANCE_TO_SECOND_CUBE_DISTANCE -
                                    yStart;
                        robot.pidDrive.setTarget(xDistance, -yDistance, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, State.START_STRAFE);
                        break;

                    case START_STRAFE:
                        robot.leftFlipper.retract();
                        robot.rightFlipper.retract();
                        robot.cubePickup.openClaw();
                        xStart = robot.driveBase.getXPosition();
                        xDistance = rightSwitch?
                            RobotInfo.STRAFE_TO_SECOND_CUBE_DISTANCE: -RobotInfo.STRAFE_TO_SECOND_CUBE_DISTANCE;
                        robot.cmdStrafeUntilCube.start(xDistance);
                        sm.setState(State.STRAFE_TO_SECOND_CUBE);
                        xPowerLimit = robot.encoderXPidCtrl.getOutputLimit();
                        robot.encoderXPidCtrl.setOutputLimit(0.45); // 0.5 originally, try 0.4
                        break;

                    case STRAFE_TO_SECOND_CUBE:
                        if (robot.cmdStrafeUntilCube.cmdPeriodic(elapsedTime))
                        {
                            sm.setState(State.PRECISION_STRAFE);
                        }
                        traceState = false;
                        break;

                    case PRECISION_STRAFE:
                        visionTarget = robot.getPixyTargetX();
                        lastXPosition = robot.driveBase.getXPosition();
                        if (visionTarget != null && visionTarget > RobotInfo.FIND_CUBE_X_TOLERANCE)
                        {
                            //robot.encoderXPidCtrl.setOutputLimit(0.5);
                            xDistance = visionTarget;
                            yDistance = 0.0;
                            robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                            sm.waitForSingleEvent(event, State.START_SECOND_PICKUP);
                        }
                        else
                        {
                            sm.setState(State.START_SECOND_PICKUP);
                        }
                        break;

                    case START_SECOND_PICKUP:
                        double xError = visionTarget != null?
                            visionTarget - (robot.driveBase.getXPosition() - lastXPosition): 0.0;
                        robot.tracer.traceInfo(funcName, "visionStrafeError=%.1f", xError);
                        visionTarget = robot.getPixyTargetX();
                        if (visionTarget == null)
                        {
                            robot.tracer.traceInfo(funcName, "Vision Target not found");
                        }
                        else
                        {
                            robot.tracer.traceInfo(funcName, "visionTargetX=%.1f", robot.getPixyTargetX());
                        }
                        robot.encoderXPidCtrl.setOutputLimit(xPowerLimit);
                        // Go forward to grab the cube or until it passes a certain distance.
                        yStart = robot.driveBase.getYPosition();
                        robot.cmdAutoCubePickup.start(xError);
                        sm.setState(State.PICKUP_SECOND_CUBE);
                        yPowerLimit = robot.encoderYPidCtrl.getOutputLimit();
                        robot.encoderYPidCtrl.setOutputLimit(0.3);
                        break;

                    case PICKUP_SECOND_CUBE:
                        if(robot.cmdAutoCubePickup.cmdPeriodic(elapsedTime))
                        {
                            sm.setState(State.BACKUP_WITH_SECOND_CUBE);
                        }
                        traceState = false;
                        break;

                    case BACKUP_WITH_SECOND_CUBE:
                        cubeStrafeDistance = robot.driveBase.getXPosition() - xStart;
                        robot.encoderYPidCtrl.setOutputLimit(yPowerLimit);
                        xDistance = 0.0;
                        yDistance = robot.driveBase.getYPosition() - yStart;
                        robot.pidDrive.setTarget(xDistance, -yDistance, robot.targetHeading, false, event);
                        robot.elevator.setPosition(RobotInfo.ELEVATOR_OFF_GROUND, event, 0.0);
                        sm.waitForSingleEvent(event, State.DONE, 1.5);
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

                if (traceState)
                {
                    robot.traceStateInfo(elapsedTime, state.toString());
                }
            }
        }

        return done;
    } // cmdPeriodic

    private void sonarTriggerEvent(int currZone, int prevZone, double zoneValue)
    {
        robot.tracer.traceInfo("SonarTrigger", "[%.3f] prevZone=%d, currZone=%d, distance=%.2f",
            Robot.getModeElapsedTime(), prevZone, currZone, zoneValue);
        if (prevZone == 1 && currZone == 0)
        {
            // Detected the switch fence.
            sonarEvent.set(true);
            if (robot.pidDrive.isActive())
            {
                robot.pidDrive.cancel();
            }
        }
    }

} // class CmdPowerUpAuto

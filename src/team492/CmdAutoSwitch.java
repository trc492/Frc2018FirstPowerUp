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

class CmdAutoSwitch implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoSwitch";

    //headings for starting backwards
    private static double DRIVE_HEADING_NORTH = 180.0;
    private static double DRIVE_HEADING_EAST = -90.0;
    private static double DRIVE_HEADING_WEST = 90.0;
    private static double DRIVE_HEADING_SOUTH = 0.0;

    //headings for starting forwards
    private static final double SWITCH_HEADING = 30.0;

    //TODO: move these to RobotInfo
    private static final double SHORTEST_DISTANCE_TO_SWITCH = 121.24;
    private static final double FAST_DELIVERY_DRIVE_PAST_SWITCH_DISTANCE = 54.0;
    private double[] sonarTriggerPoints = {8.0, 32.0};

    private static enum State
    {
        DO_DELAY,
        DRIVE_DIAGONAL_TO_SWITCH,
        THROW_CUBE,
        TURN_TO_END_OF_SWITCH,
        FAST_DELIVERY_DRIVE_PAST_SWITCH,
        DRIVE_FORWARD_DISTANCE,
        CHECK_SONAR_DISTANCE,
        SONAR_STRAFE_TO_SWITCH,
        SONAR_FLIP_CUBE,
        SONAR_POSITION_TO_STRAFE,
        TURN_TO_SWITCH,
        MOVE_ACROSS,
        STRAFE_TO_SWITCH,
        FLIP_CUBE,
        STRAFE_FROM_SWITCH,
        DRIVE_PAST_SWITCH,
        TURN_SOUTH,
        POSITION_TO_STRAFE,
        START_STRAFE,
        STRAFE_TO_SECOND_CUBE,
        PRECISION_STRAFE,
        START_SECOND_PICKUP,
        PICKUP_SECOND_CUBE,
        BACKUP_WITH_SECOND_CUBE,
        REPOSITION_TURN,
        DRIVE_TO_SECOND_TARGET,
        TURN_ROBOT,
        RAISE_ELEVATOR,
        APPROACH_FINAL_TARGET,
        DEPOSIT_CUBE,
        BACK_UP_A_BIT,
        DONE
    } // enum State

    private Robot robot;
    private double delay;
    private double forwardDistance;
    private double startPosition;
    private boolean flipInFlight = false;
    private boolean fastDeliveryFromCenter = false;

    private boolean rightSwitch;
    private boolean rightScale;
    private double switchLocation;

    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private TrcAnalogTrigger<TrcAnalogInput.DataType> sonarTrigger = null;
    private TrcEvent sonarEvent;
    private double xPowerLimit, yPowerLimit;

    private double xStart, yStart;
    private double cubeStrafeDistance;
    private Double visionTarget;
    private double sonarDistance;

    CmdAutoSwitch(Robot robot, double delay, double forwardDistance, double startPosition, boolean fastDelivery)
    {
        this.robot = robot;
        this.delay = delay;
        // if forwardDistance is -1, it means the driver picked "custom".
        this.forwardDistance = forwardDistance != -1.0?
            forwardDistance: HalDashboard.getNumber("Auto/Forward Distance", 10.0);
        this.startPosition = startPosition;

        if (startPosition == 0.0)
        {
            // when starting in the center with fast delivery, the robot is facing north.
            this.fastDeliveryFromCenter = fastDelivery;
            if (fastDeliveryFromCenter)
            {
                DRIVE_HEADING_NORTH = 0.0;
                DRIVE_HEADING_EAST = 90.0;
                DRIVE_HEADING_WEST = -90.0;
                DRIVE_HEADING_SOUTH = 180.0;
            }
        }
        else
        {
            //flipInFlight is fastDelivery for side start positions
            this.flipInFlight = fastDelivery;
        }

        this.rightSwitch = robot.gameSpecificMessage.charAt(0) == 'R';
        this.rightScale = robot.gameSpecificMessage.charAt(1) == 'R';
        this.switchLocation = rightSwitch? RobotInfo.RIGHT_SWITCH_LOCATION: RobotInfo.LEFT_SWITCH_LOCATION;

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

        robot.gyroTurnPidCtrl.setNoOscillation(true);

        robot.tracer.traceInfo(moduleName,
            "alliance=%s, gameSpecificMsg=%s, delay=%.3f, fwdDistance=%.0f, startPosition=%.1f, flipInFlight=%b",
             robot.alliance, robot.gameSpecificMessage, delay, forwardDistance, startPosition, flipInFlight);
    } // CmdAutoSwitch

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        final String funcName = "AutoSwitchPeriodic";

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
                        nextState = fastDeliveryFromCenter?
                            State.DRIVE_DIAGONAL_TO_SWITCH: State.DRIVE_FORWARD_DISTANCE;
                        if (delay == 0.0)
                        {
                            sm.setState(nextState);
                        }
                        else
                        {
                            timer.set(delay, event);
                            sm.waitForSingleEvent(event, nextState);
                        }
                        break;

                    case DRIVE_DIAGONAL_TO_SWITCH:
                        xDistance = 0.0;
                        yDistance = SHORTEST_DISTANCE_TO_SWITCH;
                        robot.targetHeading = rightSwitch? SWITCH_HEADING: -SWITCH_HEADING;
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 0.0);
                        robot.elevator.setPosition(RobotInfo.ELEVATOR_SWITCH_HEIGHT);
                        sm.waitForSingleEvent(event, State.THROW_CUBE);
                        break;

                    case THROW_CUBE:
                        robot.cubePickup.dropCube(1.0);
                        timer.set(0.3, event);
                        sm.waitForSingleEvent(event, State.TURN_TO_END_OF_SWITCH);
                        break;

                    case TURN_TO_END_OF_SWITCH:
                        xDistance = yDistance = 0.0;
                        robot.targetHeading = rightSwitch? DRIVE_HEADING_EAST: DRIVE_HEADING_WEST;
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 0.0);
                        sm.waitForSingleEvent(event, State.FAST_DELIVERY_DRIVE_PAST_SWITCH);
                        break;

                    case FAST_DELIVERY_DRIVE_PAST_SWITCH:
                        xDistance = 0.0;
                        yDistance = FAST_DELIVERY_DRIVE_PAST_SWITCH_DISTANCE;
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 0.0);
                        sm.waitForSingleEvent(event, State.TURN_SOUTH);
                        break;

                    case DRIVE_FORWARD_DISTANCE:
                        xDistance = 0.0;
                        if (switchLocation == startPosition)
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
                                // Need to slow down sooner so the cube doesn't fly over the switch.
                                sonarTrigger.setTaskEnabled(true);
                                sm.addEvent(sonarEvent);
                                yDistance -= 16.0;
                            }
                            nextState = State.CHECK_SONAR_DISTANCE;
                        }
                        else
                        {
                            //
                            // Go forward, turn and cross to the other side.
                            //
                            yDistance = forwardDistance;
                            nextState = State.TURN_TO_SWITCH;
                        }
                        robot.cubePickup.deployPickup();
                        // We are actually moving backward because we start by parking backwards,
                        // so make yDistance negative.
                        robot.pidDrive.setTarget(xDistance, -yDistance, robot.targetHeading, false, event, 0.0);
                        sm.addEvent(event);
                        sm.waitForEvents(nextState);
                        break;

                    case CHECK_SONAR_DISTANCE:
                        sonarDistance = rightSwitch? robot.getRightSonarDistance(): robot.getLeftSonarDistance();
                        // Since we moved backward, yPos is negative, let's make yStart positive.
                        yStart = -robot.driveBase.getYPosition();
                        robot.tracer.traceInfo(funcName, "sonarDistance=%.1f, yPos=%.1f", sonarDistance, yStart);

                        if (flipInFlight)
                        {
                            sonarTrigger.setTaskEnabled(false);
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
                            yDistance = RobotInfo.AUTO_DISTANCE_TO_SWITCH - yStart;
                            robot.pidDrive.setTarget(xDistance, -yDistance, robot.targetHeading, false, event, 0.0);
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
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 0.0);
                        sm.waitForSingleEvent(event, State.SONAR_FLIP_CUBE);
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

                    case SONAR_POSITION_TO_STRAFE:
                        xDistance = 0.0;
                        yDistance = RobotInfo.AUTO_DISTANCE_TO_SWITCH + RobotInfo.ADVANCE_TO_SECOND_CUBE_DISTANCE -
                                    yStart;
                        robot.pidDrive.setTarget(xDistance, -yDistance, robot.targetHeading, false, event, 0.0);
                        sm.waitForSingleEvent(event, State.START_STRAFE);
                        break;

                    case TURN_TO_SWITCH:
                        //
                        // Crossing to the other side, determine which way to turn
                        //
                        xDistance = yDistance = 0.0;
                        robot.targetHeading = rightSwitch? DRIVE_HEADING_EAST: DRIVE_HEADING_WEST;
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, State.MOVE_ACROSS);
                        break;

                    case MOVE_ACROSS:
                        if ((rightSwitch && forwardDistance == RobotInfo.FWD_DISTANCE_3) ||
                            (!rightSwitch && forwardDistance != RobotInfo.FWD_DISTANCE_3))
                        {
                            robot.rightSonarArray.startRanging(true);
                        }
                        else
                        {
                            robot.leftSonarArray.startRanging(true);
                        }
                        xDistance = 0.0;
                        yDistance = Math.abs(switchLocation - startPosition) - RobotInfo.OPPOSITE_SWITCH_OVERSHOOT;
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 0.0);
                        sm.waitForSingleEvent(event, State.STRAFE_TO_SWITCH);
                        break;

                    case STRAFE_TO_SWITCH:
                        // Changing the first trigger point to zero so the threshold is about 16-inch.
                        sonarTriggerPoints[0] = 0.0;
                        sonarTrigger.setTaskEnabled(true);
                        if(forwardDistance != RobotInfo.FWD_DISTANCE_3)sm.addEvent(sonarEvent);
                        yDistance = 0.0;
                        // questionable strafe calculation, needs fixing
                        if(forwardDistance > RobotInfo.AUTO_DISTANCE_TO_SWITCH)
                        {
                            xDistance = -(forwardDistance - RobotInfo.AUTO_DISTANCE_TO_SWITCH - RobotInfo.CUBE_WIDTH-53.0);
                        } else
                        {
                            xDistance = (RobotInfo.AUTO_DISTANCE_TO_SWITCH - forwardDistance) - 36.0;
                        }
                        if (rightSwitch) xDistance = -xDistance;
                        robot.tracer.traceInfo(moduleName, "Strafe to switch, forwardDistance=%.1f,xDistance=%.2f",
                            forwardDistance, xDistance);
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 0.0);
                        sm.addEvent(event);
                        sm.waitForEvents(State.FLIP_CUBE);
                        break;

                    case FLIP_CUBE:
                        robot.rightSonarArray.stopRanging();
                        robot.leftSonarArray.stopRanging();
                        sonarTrigger.setTaskEnabled(false);
                        if ((rightSwitch && forwardDistance == RobotInfo.FWD_DISTANCE_3) ||
                            (!rightSwitch && forwardDistance != RobotInfo.FWD_DISTANCE_3))
                        {
                            robot.rightFlipper.extend();
                        }
                        else
                        {
                            robot.leftFlipper.extend();
                        }
                        timer.set(0.3, event);
                        nextState = forwardDistance == RobotInfo.FWD_DISTANCE_3?
                            State.STRAFE_FROM_SWITCH: State.DRIVE_PAST_SWITCH;
                        sm.waitForSingleEvent(event, nextState);
                        break;

                    case STRAFE_FROM_SWITCH:
                        yDistance = 0.0;
                        xDistance = -RobotInfo.STRAFE_FROM_SWITCH_DISTANCE;
                        if (!rightSwitch) xDistance = -xDistance;
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 0.0);
                        sm.waitForSingleEvent(event, State.TURN_SOUTH);
                        break;

                    case DRIVE_PAST_SWITCH:
                        xDistance = 0.0;
                        yDistance = RobotInfo.OPPOSITE_SWITCH_OVERSHOOT + 21.0;
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 0.0);
                        sm.waitForSingleEvent(event, State.TURN_SOUTH);
                        break;

                    case TURN_SOUTH:
                        xDistance = yDistance = 0.0;
                        robot.targetHeading = DRIVE_HEADING_SOUTH;
                        nextState = (forwardDistance == RobotInfo.FWD_DISTANCE_3 && !fastDeliveryFromCenter)?
                            State.PRECISION_STRAFE: State.POSITION_TO_STRAFE;
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, nextState);
                        break;

                    case POSITION_TO_STRAFE:
                        xDistance = 0.0;
                        yDistance = RobotInfo.POSITION_TO_STRAFE_DISTANCE;
                        robot.pidDrive.setTarget(xDistance, -yDistance, robot.targetHeading, false, event, 0.0);
                        sm.waitForSingleEvent(event, State.START_STRAFE);
                        break;

                    case START_STRAFE:
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
                        robot.leftFlipper.retract();
                        robot.rightFlipper.retract();
                        visionTarget = robot.getPixyTargetX();
                        xStart = robot.driveBase.getXPosition();
                        if (visionTarget != null && visionTarget > RobotInfo.FIND_CUBE_X_TOLERANCE)
                        {
                            //robot.encoderXPidCtrl.setOutputLimit(0.5);
                            xDistance = visionTarget;
                            yDistance = 0.0;
                            robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 0.0);
                            sm.waitForSingleEvent(event, State.START_SECOND_PICKUP);
                        }
                        else
                        {
                            sm.setState(State.START_SECOND_PICKUP);
                        }
                        break;

                    case START_SECOND_PICKUP:
                        double xError = visionTarget != null?
                            visionTarget - (robot.driveBase.getXPosition() - xStart): 0.0;
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
                        robot.pidDrive.setTarget(xDistance, -yDistance, robot.targetHeading, false, event, 0.0);
                        robot.elevator.setPosition(RobotInfo.ELEVATOR_CRUISE_HEIGHT);
                        sm.waitForSingleEvent(event, State.REPOSITION_TURN);
                        break;

                    case REPOSITION_TURN:
                        xDistance = yDistance = 0.0;
                        if(rightScale == rightSwitch)
                        {
                            robot.targetHeading = DRIVE_HEADING_NORTH;
                            nextState = State.RAISE_ELEVATOR;
                        }
                        else
                        {
                            robot.targetHeading = rightScale? DRIVE_HEADING_EAST: DRIVE_HEADING_WEST;
                            nextState = State.DRIVE_TO_SECOND_TARGET;
                        }
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, nextState);
                        break;

                    case DRIVE_TO_SECOND_TARGET:
                        xDistance = 0;
                        yDistance =
                            RobotInfo.SCALE_FRONT_POSITION + (RobotInfo.RIGHT_START_POS - cubeStrafeDistance);
                        robot.tracer.traceInfo(funcName, "cubeStrafeDistance=%.1f driveAcrossFieldDistance=%.1f",cubeStrafeDistance, yDistance);
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 0.0);
                        sm.waitForSingleEvent(event, State.TURN_ROBOT);
                        break;

                    case TURN_ROBOT:
                        xDistance = yDistance = 0.0;
                        robot.targetHeading = DRIVE_HEADING_NORTH;
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, State.RAISE_ELEVATOR);
                        break;

                    case RAISE_ELEVATOR:
                        robot.tracer.traceInfo(funcName, "ElevatorStartHeight=%.1f", robot.elevator.getPosition());
                        robot.elevator.setPosition(RobotInfo.ELEVATOR_SCALE_HIGH, event, 3.0);
                        sm.waitForSingleEvent(event, State.APPROACH_FINAL_TARGET);
                        break;

                    case APPROACH_FINAL_TARGET:
                        // Do another setPosition without event so it will hold position.
                        robot.elevator.setPosition(RobotInfo.ELEVATOR_SCALE_HIGH);
                        robot.tracer.traceInfo(funcName, "ElevatorStopHeight=%.1f", robot.elevator.getPosition());
                        xDistance = 0.0;
                        // left this side approach distance just in case we need it again
                        yDistance = RobotInfo.FINAL_FRONT_SCALE_APPROACH_DISTANCE;
                        yPowerLimit = robot.encoderYPidCtrl.getOutputLimit();
                        robot.encoderYPidCtrl.setOutputLimit(0.5);
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 0.0);
                        sm.waitForSingleEvent(event, State.DEPOSIT_CUBE);
                        break;

                    case DEPOSIT_CUBE:
                        robot.encoderYPidCtrl.setOutputLimit(yPowerLimit);
                        robot.cubePickup.dropCube(1.0);
                        timer.set(0.3, event);
                        sm.waitForSingleEvent(event, State.BACK_UP_A_BIT);
                        break;
                        
                    case BACK_UP_A_BIT:
                        xDistance = 0.0;
                        yDistance = -15.0;
                        robot.cubePickup.stopPickup();
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 0.0);
                        sm.waitForSingleEvent(event, State.DONE);
                        
                       
                    case DONE:
                        //
                        // We are done.
                        //
                        robot.elevator.setPosition(RobotInfo.ELEVATOR_MIN_HEIGHT);
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

} // class CmdAutoSwitch

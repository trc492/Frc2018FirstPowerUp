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

import trclib.TrcAnalogInput;
import trclib.TrcAnalogTrigger;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class CmdAutoSideSwitch implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoSideSwitch";

    private static final double NORTH_HEADING = 0.0;
    private static final double SOUTH_HEADING = 180.0;
    private static final double EAST_HEADING = 90.0;
    private static final double WEST_HEADING = -90.0;

    private static final double FAST_DELIVERY_Y_TOLERANCE = 5.0;
    private static final double DRIVE_PAST_SWITCH_DISTANCE = 45.0;
    private static final double GET_TO_SWITCH_DISTANCE = 50.0;

    private static final double[] sonarTriggerPoints = { 8.0, 32.0 };

    private static enum State
    {
        DO_DELAY,
        DRIVE_TO_SWITCH,
        TURN_AND_TOUCH_SWITCH,
        DROP_CUBE,
        TURN_SOUTH,
        DRIVE_PAST_SWITCH,
        START_STRAFE,
        STRAFE_TO_SECOND_CUBE,
        PRECISION_STRAFE,
        START_SECOND_PICKUP,
        PICKUP_SECOND_CUBE,
        RAISE_ELEVATOR_FOR_SECOND_SWITCH,
        DROP_SECOND_CUBE,
        BACKUP_WITH_CUBE,
        REPOSITION_TURN,
        RAISE_ELEVATOR_FOR_SCALE,
        APPROACH_FINAL_TARGET,
        DEPOSIT_CUBE,
        BACK_UP_A_BIT,
        LOWER_ELEVATOR,
        DONE
    } // enum State

    private Robot robot;
    private double delay;
    private boolean getSecondCube;

    private boolean rightSwitch;
    private boolean rightScale;
    private boolean additionalSwitchCube;

    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private TrcAnalogTrigger<TrcAnalogInput.DataType> leftSonarTrigger = null;
    private TrcAnalogTrigger<TrcAnalogInput.DataType> rightSonarTrigger = null;
    private TrcEvent sonarEvent;
    private double xPowerLimit, yPowerLimit;

    private double xStart, yStart;
    private Double visionTarget;
    private boolean hasDroppedSecondCube = false;
    private boolean useSonar;

    public CmdAutoSideSwitch(Robot robot, double delay, boolean getSecondCube, boolean useSonar)
    {
        robot.globalTracer.traceInfo(moduleName, "[%.3f] delay=%.1f, getSecondCube=%b",
            Robot.getModeElapsedTime(), delay, getSecondCube);

        this.robot = robot;
        this.delay = delay;
        this.getSecondCube = getSecondCube;
        this.useSonar = useSonar;

        this.rightSwitch = robot.gameSpecificMessage.charAt(0) == 'R';
        this.rightScale = robot.gameSpecificMessage.charAt(1) == 'R';
        this.additionalSwitchCube = rightSwitch == rightScale;

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        if (delay != 0.0)
            sm.start(State.DO_DELAY);
        else
            sm.start(State.DRIVE_TO_SWITCH);

        leftSonarTrigger = new TrcAnalogTrigger<>("LeftSonarTrigger", robot.leftSonarSensor, 0,
            TrcAnalogInput.DataType.INPUT_DATA, sonarTriggerPoints, this::sonarTriggerEvent);
        rightSonarTrigger = new TrcAnalogTrigger<>("RightSonarTrigger", robot.rightSonarSensor, 0,
            TrcAnalogInput.DataType.INPUT_DATA, sonarTriggerPoints, this::sonarTriggerEvent);
        sonarEvent = new TrcEvent("SonarEvent");

        xPowerLimit = robot.encoderXPidCtrl.getOutputLimit();
        yPowerLimit = robot.encoderYPidCtrl.getOutputLimit();

        robot.globalTracer.traceInfo(moduleName, "alliance=%s, gameSpecificMsg=%s, delay=%.3f, getSecondCube=%b",
            robot.alliance, robot.gameSpecificMessage, delay, getSecondCube);
    } // CmdAutoSwitch

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        final String funcName = "AutoSideSwitchPeriodic";

        boolean done = !sm.isEnabled();

        if (!done)
        {
            State state = sm.checkReadyAndGetState();
            //
            // Print debug info.
            //
            robot.dashboard.displayPrintf(1, "State: %s", state != null ? state : sm.getState());

            if (state != null)
            {
                double xDistance, yDistance;
                State nextState;
                boolean traceState = true;

                switch (state)
                {
                    // TODO: need to make sure that all encoder power limits get
                    // set correctly in all states
                    case DO_DELAY:
                        //
                        // Do delay if any.
                        //
                        if (delay == 0.0)
                        {
                            sm.setState(State.DRIVE_TO_SWITCH);
                        }
                        else
                        {
                            timer.set(delay, event);
                            sm.waitForSingleEvent(event, State.DRIVE_TO_SWITCH);
                        }
                        break;

                    case DRIVE_TO_SWITCH:
                        robot.driveBase.setBrakeMode(false);
                        robot.encoderYPidCtrl.setNoOscillation(true);
                        robot.encoderYPidCtrl.setTargetTolerance(FAST_DELIVERY_Y_TOLERANCE);
                        robot.gyroTurnPidCtrl.setTargetTolerance(4.0);
                        if (useSonar)
                        {
                            sonarEvent.clear();
                            if (rightSwitch)
                            {
                                robot.leftSonarArray.startRanging(true);
                                leftSonarTrigger.setTaskEnabled(true);
                            }
                            else
                            {
                                robot.rightSonarArray.startRanging(true);
                                rightSonarTrigger.setTaskEnabled(true);
                            }
                            sm.addEvent(sonarEvent);
                        }
                        xDistance = 0.0;
                        yDistance = RobotInfo.AUTO_DISTANCE_TO_SWITCH - 30;
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 0.0);
                        sm.addEvent(event);
                        robot.elevator.setPosition(RobotInfo.ELEVATOR_SWITCH_HEIGHT);
                        sm.waitForEvents(State.TURN_AND_TOUCH_SWITCH);
                        break;

                    case TURN_AND_TOUCH_SWITCH:
                        yStart = robot.driveBase.getYPosition();
                        if (rightSwitch)
                        {
                            robot.leftSonarArray.stopRanging();
                            leftSonarTrigger.setTaskEnabled(false);
                            robot.targetHeading = WEST_HEADING;
                        }
                        else
                        {
                            robot.rightSonarArray.stopRanging();
                            rightSonarTrigger.setTaskEnabled(false);
                            robot.targetHeading = EAST_HEADING;
                        }
                        xDistance = 0.0;
                        yDistance = GET_TO_SWITCH_DISTANCE;
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 0.0);
                        sm.waitForSingleEvent(event, State.DROP_CUBE);
                        break;

                    case DROP_CUBE:
                        robot.driveBase.setBrakeMode(true);
                        robot.encoderYPidCtrl.setNoOscillation(false);
                        robot.encoderYPidCtrl.setTargetTolerance(RobotInfo.ENCODER_Y_TOLERANCE);
                        robot.cubePickup.openClaw();
                        robot.cubePickup.dropCube(0.54);
                        if (getSecondCube)
                        {
                            timer.set(0.3, event);
                            sm.waitForSingleEvent(event, State.TURN_SOUTH);
                        }
                        else
                        {
                            sm.setState(State.DONE);
                        }
                        break;

                    case TURN_SOUTH:
                        robot.cubePickup.stopPickup();
                        robot.elevator.setPosition(RobotInfo.ELEVATOR_MIN_HEIGHT);
                        xDistance = yDistance = 0.0;
                        robot.targetHeading = SOUTH_HEADING;
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 0.0);
                        sm.waitForSingleEvent(event, State.DRIVE_PAST_SWITCH);
                        break;

                    case DRIVE_PAST_SWITCH:
                        xDistance = 0.0;
                        yDistance = DRIVE_PAST_SWITCH_DISTANCE;
                        robot.pidDrive.setTarget(xDistance, -yDistance, robot.targetHeading, false, event, 0.0);
                        sm.waitForSingleEvent(event, State.START_STRAFE);
                        break;

                    case START_STRAFE:
                        robot.cubePickup.prepareForPickup();
                        xStart = robot.driveBase.getXPosition();
                        xDistance = rightSwitch ?
                            RobotInfo.STRAFE_TO_SECOND_CUBE_DISTANCE: -RobotInfo.STRAFE_TO_SECOND_CUBE_DISTANCE;
                        robot.cmdStrafeUntilCube.start(xDistance);
                        sm.setState(State.STRAFE_TO_SECOND_CUBE);
                        xPowerLimit = robot.encoderXPidCtrl.getOutputLimit();
                        robot.encoderXPidCtrl.setOutputLimit(0.7);
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
                        xStart = robot.driveBase.getXPosition();
                        if (visionTarget != null && Math.abs(visionTarget) > RobotInfo.FIND_CUBE_X_TOLERANCE)
                        {
                            // robot.encoderXPidCtrl.setOutputLimit(0.5);
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
                        double xError = visionTarget != null ?
                            visionTarget - (robot.driveBase.getXPosition() - xStart): 0.0;
                        robot.globalTracer.traceInfo(funcName, "visionStrafeError=%.1f", xError);
                        visionTarget = robot.getPixyTargetX();
                        if (visionTarget == null)
                        {
                            robot.globalTracer.traceInfo(funcName, "Vision Target not found");
                        }
                        else
                        {
                            robot.globalTracer.traceInfo(funcName, "visionTargetX=%.1f", robot.getPixyTargetX());
                        }
                        robot.encoderXPidCtrl.setOutputLimit(xPowerLimit);
                        // Go forward to grab the cube or until it passes a
                        // certain distance.
                        yStart = robot.driveBase.getYPosition();
                        robot.cmdAutoCubePickup.start(xError);
                        sm.setState(State.PICKUP_SECOND_CUBE);
                        yPowerLimit = robot.encoderYPidCtrl.getOutputLimit();
                        robot.encoderYPidCtrl.setOutputLimit(0.3);
                        break;

                    case PICKUP_SECOND_CUBE:
                        if (robot.cmdAutoCubePickup.cmdPeriodic(elapsedTime))
                        {
                            if (additionalSwitchCube)
                            {
                                sm.setState(State.RAISE_ELEVATOR_FOR_SECOND_SWITCH);
                            }
                            else
                            {
                                sm.setState(State.BACKUP_WITH_CUBE);
                            }
                        }
                        traceState = false;
                        break;

                    case RAISE_ELEVATOR_FOR_SECOND_SWITCH:
                        // TODO: might need to back up from switch a little
                        // before raising elevator
                        robot.elevator.setPosition(RobotInfo.ELEVATOR_SWITCH_HEIGHT, event, 1.0);
                        if (hasDroppedSecondCube)
                        {
                            sm.waitForSingleEvent(event, State.DONE);
                        }
                        else
                        {
                            sm.waitForSingleEvent(event, State.DROP_SECOND_CUBE);
                        }
                        break;

                    case DROP_SECOND_CUBE:
                        robot.cubePickup.openClaw();
                        robot.cubePickup.dropCube(0.6);
                        hasDroppedSecondCube = true;
                        timer.set(0.3, event);
                        sm.waitForSingleEvent(event, State.BACKUP_WITH_CUBE);
                        break;

                    case BACKUP_WITH_CUBE:
                        robot.cubePickup.stopPickup();
                        robot.encoderYPidCtrl.setOutputLimit(yPowerLimit);
                        xDistance = 0.0;
                        yDistance = robot.driveBase.getYPosition() - yStart;
                        robot.pidDrive.setTarget(xDistance, -yDistance, robot.targetHeading, false, event, 0.0);
                        if (additionalSwitchCube)
                        {
                            sm.waitForSingleEvent(event, State.START_STRAFE);
                            robot.elevator.setPosition(RobotInfo.ELEVATOR_MIN_HEIGHT);
                        }
                        else
                        {
                            robot.elevator.setPosition(RobotInfo.ELEVATOR_CRUISE_HEIGHT);
                            sm.waitForSingleEvent(event, State.REPOSITION_TURN);
                        }
                        break;

                    case REPOSITION_TURN:
                        xDistance = yDistance = 0.0;
                        robot.targetHeading = NORTH_HEADING;
                        nextState = State.RAISE_ELEVATOR_FOR_SCALE;
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, nextState);
                        break;

                    case RAISE_ELEVATOR_FOR_SCALE:
                        robot.globalTracer.traceInfo(funcName, "ElevatorStartHeight=%.1f", robot.elevator.getPosition());
                        robot.elevator.setPosition(RobotInfo.ELEVATOR_SCALE_HIGH, event, 3.0);
                        sm.waitForSingleEvent(event, State.APPROACH_FINAL_TARGET);
                        break;

                    case APPROACH_FINAL_TARGET:
                        // Do another setPosition without event so it will hold
                        // position.
                        robot.elevator.setPosition(RobotInfo.ELEVATOR_SCALE_HIGH);
                        robot.globalTracer.traceInfo(funcName, "ElevatorStopHeight=%.1f", robot.elevator.getPosition());
                        xDistance = 0.0;
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
                        robot.cubePickup.stopPickup();
                        xDistance = 0.0;
                        yDistance = -15.0;
                        robot.cubePickup.stopPickup();
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 0.0);
                        sm.waitForSingleEvent(event, State.DONE);

                    case LOWER_ELEVATOR:
                        robot.elevator.setPosition(RobotInfo.ELEVATOR_MIN_HEIGHT);
                        break;

                    case DONE:
                        //
                        // We are done.
                        //
                        robot.cubePickup.stopPickup();
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
        robot.globalTracer.traceInfo("SonarTrigger", "[%.3f] prevZone=%d, currZone=%d, distance=%.2f",
            Robot.getModeElapsedTime(), prevZone, currZone, zoneValue);

        if (Robot.getModeElapsedTime() <= 1.0)
            return;

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

} // class CmdAutoSideSwitch

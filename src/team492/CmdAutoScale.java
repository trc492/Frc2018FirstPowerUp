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

import frclib.FrcAnalogInput;
import team492.RobotInfo.Position;
import trclib.TrcAnalogInput;
import trclib.TrcAnalogTrigger;
import trclib.TrcEvent;
import trclib.TrcMaxbotixSonarArray;
import trclib.TrcStateMachine;
import trclib.TrcTimer;
import trclib.TrcRobot;

public class CmdAutoScale implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoScale";
    private static final double DRIVE_HEADING_NORTH = 0.0;
    private static final double DRIVE_HEADING_EAST = 90.0;
    private static final double DRIVE_HEADING_WEST = -90.0;
    private static final double[] distances = new double[] { 10.0, 50.0 };

    private enum State
    {
        START,
        DRIVE_TO_LANE,
        TURN_TO_DRIVE_ACROSS_FIELD,
        DRIVE_ACROSS_FIELD,
        TURN_NORTH,
        DRIVE_TO_SWITCH,
        CHECK_SONAR_DISTANCE,
        TURN_TO_OPPOSITE_SCALE,
        DRIVE_TO_OPPOSITE_SCALE,
        TURN_TO_SCALE,
        FINISH_DRIVE_TO_SCALE,
        TURN_TO_FACE_SCALE,
        RAISE_ELEVATOR,
        THROW_CUBE,
        DONE
    }

    private Robot robot;
    private double delay;
    private Position startPosition;
    private double forwardDriveDistance;
    private TrcEvent event, sonarEvent, elevatorEvent;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private double startY;
    private boolean startRight;
    private boolean scaleRight;
    private boolean sameSide;
    private boolean lane3;
    private TrcAnalogTrigger<TrcAnalogInput.DataType> sonarTrigger;
    private TrcMaxbotixSonarArray sonarArray;

    private double distanceFromWall = 0.0;
    private double sonarDistance = 0.0;

    /**
     * 
     * @param robot Robot class
     * @param delay How much to delay (in seconds) before starting
     * @param startPosition Either RobotInfo.LEFT_START_POS, RobotInfo.MID_START_POS, or RobotInfo.RIGHT_START_POS
     * @param forwardDriveDistance
     */
    public CmdAutoScale(Robot robot, double delay, Position startPosition, double forwardDriveDistance)
    {
        robot.globalTracer.traceInfo(moduleName, "[%.3f] delay=%.1f, startPos=%s, fwdDistance=%.1f",
            Robot.getModeElapsedTime(), delay, startPosition, forwardDriveDistance);

        this.robot = robot;
        this.delay = delay;
        this.startPosition = startPosition;

        robot.gyroTurnPidCtrl.setTargetTolerance(3.0);

        this.forwardDriveDistance = forwardDriveDistance;
        event = new TrcEvent(moduleName);
        sonarEvent = new TrcEvent(moduleName + ".sonarEvent");
        elevatorEvent = new TrcEvent(moduleName + ".elevatorEvent");
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);

        startRight = this.startPosition == Position.RIGHT_POS;
        scaleRight = robot.gameSpecificMessage.charAt(1) == 'R';
        sameSide = startRight == scaleRight;
        lane3 = forwardDriveDistance > RobotInfo.AUTO_DISTANCE_TO_SWITCH;

        FrcAnalogInput sonarSensor;
        if (scaleRight && sameSide ||
            scaleRight && !sameSide && !lane3 ||
            !scaleRight && !sameSide && lane3)
        {
            sonarArray = robot.leftSonarArray;
            sonarSensor = robot.leftSonarSensor;
        }
        else
        {
            sonarArray = robot.rightSonarArray;
            sonarSensor = robot.rightSonarSensor;
        }

        sonarTrigger = new TrcAnalogTrigger<>(
            "SwitchDistanceTrigger", sonarSensor,
            0, TrcAnalogInput.DataType.INPUT_DATA, distances, this::sonarTriggerEvent);

        robot.gyroTurnPidCtrl.setNoOscillation(true);

        robot.globalTracer.traceInfo(moduleName,
            "alliance=%s, gameSpecificMsg=%s, delay=%.3f, startPosition=%s, fwdDistance=%.0f",
             robot.alliance, robot.gameSpecificMessage, delay, startPosition, forwardDriveDistance);
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();

        if (done || startPosition == Position.MID_POS) return true;

        State state = sm.checkReadyAndGetState();

        //
        // Print debug info.
        //
        robot.dashboard.displayPrintf(1, "State: %s", state != null? state: sm.getState());

        if (state != null)
        {
            double xDistance, yDistance;
            State nextState;

            switch(state)
            {
                case START:
                    nextState = (sameSide || lane3)? State.DRIVE_TO_SWITCH: State.DRIVE_TO_LANE;
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

                case DRIVE_TO_LANE:
                    // forwardDriveDistance is set to lane 1 or lane 2.
                    // We are going across in front of the switch.
                    robot.pidDrive.setTarget(0.0, forwardDriveDistance, robot.targetHeading, false, event, 0.0);
                    sm.waitForSingleEvent(event, State.TURN_TO_DRIVE_ACROSS_FIELD);
                    break;

                case TURN_TO_DRIVE_ACROSS_FIELD:
                    robot.targetHeading = startRight?DRIVE_HEADING_WEST:DRIVE_HEADING_EAST;
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_ACROSS_FIELD);
                    break;

                case DRIVE_ACROSS_FIELD:
                    robot.pidDrive.setTarget(
                        0.0, RobotInfo.DRIVE_ACROSS_FIELD_DISTANCE, robot.targetHeading, false, event, 0.0);
                    sm.waitForSingleEvent(event, State.TURN_NORTH);
                    break;

                case TURN_NORTH:
                    robot.targetHeading = DRIVE_HEADING_NORTH;
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_SWITCH);
                    break;

                case DRIVE_TO_SWITCH:
                    // We come here either we started on the same side or we are crossing to the other side in lane 3.
                    // Or, we crossed from the other side in front of the switch to here.
                    setRangingEnabled(true);
                    setSonarTriggerEnabled(true);

                    startY = robot.driveBase.getYPosition();
                    yDistance = RobotInfo.AUTO_DISTANCE_TO_SWITCH + 24.0;
                    // We came from the other side in lane 1 or lane 2 so need to subtract forwardDriveDistance.
                    if (!sameSide && !lane3) yDistance -= forwardDriveDistance;

                    robot.pidDrive.setTarget(0.0, yDistance, robot.targetHeading, false, event, 0.0);
                    sm.addEvent(event);
                    sm.addEvent(sonarEvent);
                    sm.waitForEvents(State.CHECK_SONAR_DISTANCE);
                    break;

                case CHECK_SONAR_DISTANCE:
                    // We are going to range the switch to center ourselves in the corridor moving forward.
                    setSonarTriggerEnabled(false);
                    robot.pidDrive.cancel();
                    sonarDistance = sonarArray.getDistance(0).value;
                    distanceFromWall = RobotInfo.SWITCH_TO_WALL_DISTANCE - sonarDistance - RobotInfo.ROBOT_WIDTH/2.0;

                    double currY = robot.driveBase.getYPosition();
                    if (!sameSide && !lane3) currY = currY - startY + forwardDriveDistance;
                    if (sameSide || !lane3)
                    {
                        xDistance = RobotInfo.SCALE_TO_WALL_DISTANCE - RobotInfo.ROBOT_TO_SCALE_DISTANCE - distanceFromWall;
                        if (scaleRight) xDistance *= -1;
                        // Start raising elevator
                        robot.elevator.setPosition(RobotInfo.ELEVATOR_CRUISE_HEIGHT);
                        // The target scale is ahead of us, go to it.
                        yDistance = RobotInfo.FIELD_LENGTH/2.0 - currY - RobotInfo.ROBOT_LENGTH/2.0;
                        nextState = State.TURN_TO_FACE_SCALE;
                    }
                    else
                    {
                        // The target scale is on the other side, go to lane 3 to cross over.
                        xDistance = 0.0;
                        yDistance = forwardDriveDistance - currY;
                        nextState = State.TURN_TO_OPPOSITE_SCALE;
                    }

                    robot.globalTracer.traceInfo(moduleName, "sonarDistance=%.1f, distanceFromWall=%.1f, xDistance=%.1f, yDistance=%.1f",
                        sonarDistance, distanceFromWall, xDistance, yDistance);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 0.0);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case TURN_TO_OPPOSITE_SCALE:
                    robot.targetHeading = startRight?DRIVE_HEADING_WEST:DRIVE_HEADING_EAST;
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_OPPOSITE_SCALE);
                    break;

                case DRIVE_TO_OPPOSITE_SCALE:
                    yDistance = RobotInfo.FIELD_WIDTH - distanceFromWall - RobotInfo.SCALE_TO_WALL_DISTANCE + RobotInfo.ROBOT_TO_SCALE_DISTANCE;
                    robot.pidDrive.setTarget(0.0, yDistance, robot.targetHeading, false, event, 0.0);
                    sm.waitForSingleEvent(event, State.TURN_TO_SCALE);
                    break;

                case TURN_TO_SCALE:
                    robot.targetHeading = DRIVE_HEADING_NORTH;
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.FINISH_DRIVE_TO_SCALE);
                    break;

                case FINISH_DRIVE_TO_SCALE:
                    yDistance = RobotInfo.ALLIANCE_WALL_TO_SCALE_DISTANCE - forwardDriveDistance - RobotInfo.ROBOT_LENGTH/2.0 + 10.0;
                    robot.pidDrive.setTarget(0.0, yDistance, robot.targetHeading, false, event, 0.0);
                    // Start raising elevator
                    robot.elevator.setPosition(RobotInfo.ELEVATOR_CRUISE_HEIGHT);
                    sm.waitForSingleEvent(event, State.TURN_TO_FACE_SCALE);
                    break;

                case TURN_TO_FACE_SCALE:
                    robot.gyroTurnPidCtrl.setNoOscillation(false);
                    robot.targetHeading = scaleRight?DRIVE_HEADING_WEST:DRIVE_HEADING_EAST;
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.RAISE_ELEVATOR);
                    break;

                case RAISE_ELEVATOR:
                    // Already started to raise elevator, so wait for it to complete
                    robot.elevator.setPosition(RobotInfo.ELEVATOR_SCALE_HIGH - 18.0, elevatorEvent, 0.0);
                    sm.waitForSingleEvent(elevatorEvent, State.THROW_CUBE);
                    break;

                case THROW_CUBE:
                    // Do another setPosition without event so it will hold position.
                    robot.elevator.setPosition(RobotInfo.ELEVATOR_SCALE_HIGH - 18.0);
                    robot.cubePickup.deployPickup();
                    robot.cubePickup.dropCube(RobotInfo.CUBE_PICKUP_DROP_POWER);
                    timer.set(RobotInfo.DROP_CUBE_TIMEOUT, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                    robot.cubePickup.stopPickup();
                    robot.elevator.setPosition(RobotInfo.ELEVATOR_MIN_HEIGHT);
                    done = true;
                    setSonarTriggerEnabled(false);
                    setRangingEnabled(false);
                    sm.stop();
                    robot.gyroTurnPidCtrl.setNoOscillation(false);
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString());
        }
        return done;
    }

    private void setSonarTriggerEnabled(boolean enabled)
    {
        robot.globalTracer.traceInfo(moduleName, "setSonarTriggerEnabled(%b)", enabled);
        sonarTrigger.setTaskEnabled(enabled);
        if (enabled) sonarEvent.clear();
    }

    private void setRangingEnabled(boolean enabled)
    {
        robot.globalTracer.traceInfo(moduleName, "setRangingEnabled(%b)", enabled);
        if(enabled)
        {
            sonarArray.startRanging(true);
        }
        else
        {
            sonarArray.stopRanging();
        }
    }

    private void sonarTriggerEvent(int currZone, int prevZone, double zoneValue)
    {
        robot.globalTracer.traceInfo(moduleName, "SonarTriggerEvent: prevZone=%d, currZone=%d, value:%.1f",
            prevZone, currZone, zoneValue);

        if (prevZone == 1 && currZone == 0)
        {
            sonarEvent.set(true);
        }
    }

}

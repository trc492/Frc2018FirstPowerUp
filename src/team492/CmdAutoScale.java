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
import trclib.TrcStateMachine;
import trclib.TrcTimer;
import trclib.TrcRobot;

public class CmdAutoScale implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoScale";
    private static final double DRIVE_HEADING_NORTH = 0.0;
    private static final double DRIVE_HEADING_EAST = 90.0;
    private static final double DRIVE_HEADING_WEST = -90.0;
    private static final double[] distances = new double[] { 10.0, 60.0 };

    private enum State
    {
        START,
        DRIVE_TO_LANE,
        TURN_NORTH,
        DRIVE_TO_SWITCH,
        DRIVE_TO_LANE_3,
        TURN_TO_OPPOSITE_SCALE,
        DRIVE_TO_OPPOSITE_SCALE,
        TURN_TO_SCALE,
        FINISH_DRIVE_TO_SCALE,
        DRIVE_TO_SCALE,
        TURN_TO_FACE_SCALE,
        RAISE_ELEVATOR,
        THROW_CUBE,
        LOWER_ELEVATOR,
        DONE
    }

    private enum Position
    {
        LEFT, MIDDLE, RIGHT
    }

    private Robot robot;
    private double delay;
    private TrcEvent event, sonarEvent, elevatorEvent;
    private TrcStateMachine<State> sm;
    private TrcTimer timer;
    private Position startPosition;
    private boolean startRight;
    private boolean scaleRight;
    private double forwardDriveDistance;
    private TrcAnalogTrigger<TrcAnalogInput.DataType> sonarTrigger;

    private double distanceFromWall = 0.0;
    private double sonarDistance = 0.0;

    /**
     * 
     * @param robot Robot class
     * @param delay How much to delay (in seconds) before starting
     * @param startPosition Either RobotInfo.LEFT_START_POS, RobotInfo.MID_START_POS, or RobotInfo.RIGHT_START_POS
     * @param forwardDriveDistance
     */
    public CmdAutoScale(Robot robot, double delay, double startPosition, double forwardDriveDistance)
    {
        this.robot = robot;
        this.delay = delay;
        this.forwardDriveDistance = forwardDriveDistance;
        event = new TrcEvent(moduleName);
        sonarEvent = new TrcEvent(moduleName + ".sonarEvent");
        elevatorEvent = new TrcEvent(moduleName + ".elevatorEvent");
        sm = new TrcStateMachine<>(moduleName);
        timer = new TrcTimer(moduleName);

        if (startPosition == RobotInfo.LEFT_START_POS) this.startPosition = Position.LEFT;
        else if (startPosition == RobotInfo.RIGHT_START_POS) this.startPosition = Position.RIGHT;
        else this.startPosition = Position.MIDDLE;

        startRight = this.startPosition == Position.RIGHT;
        scaleRight = robot.gameSpecificMessage.charAt(1) == 'R';

        sonarTrigger = new TrcAnalogTrigger<>(
            "SwitchDistanceTrigger", startRight? robot.leftSonarSensor: robot.rightSonarSensor,
            0, TrcAnalogInput.DataType.INPUT_DATA, distances, this::sonarTriggerEvent);

        sm.start(State.START);

        robot.tracer.traceInfo(moduleName,
            "alliance=%s, gameSpecifiMsg=%s, delay=%.3f, startPosition=%.1f",
             robot.alliance, robot.gameSpecificMessage, delay, startPosition);
    }

    public void setSonarTriggerEnabled(boolean enabled)
    {
        robot.tracer.traceInfo(moduleName, "setSonarTriggerEnabled(%b)", enabled);
        sonarTrigger.setTaskEnabled(enabled);
        if (enabled) sonarEvent.clear();
    }
    
    public void setRangingEnabled(boolean enabled)
    {
        if (startRight && enabled)
        {
            robot.leftSonarArray.startRanging(true);
        }
        else if (!startRight && enabled)
        {
            robot.rightSonarArray.startRanging(true);
        }
        else if (!enabled)
        {
            robot.leftSonarArray.stopRanging();
            robot.rightSonarArray.stopRanging();
        }
    }

    private void sonarTriggerEvent(int currZone, int prevZone, double zoneValue)
    {
        robot.tracer.traceInfo(moduleName, "SonarTriggerEvent: prevZone=%d, currZone=%d, value:%.1f",
            prevZone, currZone, zoneValue);

        if (prevZone == 1 && currZone == 0)
        {
            sonarEvent.set(true);
        }
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();
        if (done || startPosition == Position.MIDDLE) return true;

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
                    if (delay != 0.0)
                    {
                        sm.setState(State.DRIVE_TO_SWITCH);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_TO_SWITCH);
                    }
                    break;

                case DRIVE_TO_LANE:
                    robot.pidDrive.setTarget(0.0, forwardDriveDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_NORTH);
                    break;

                case TURN_NORTH:
                    robot.targetHeading = DRIVE_HEADING_NORTH;
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_SWITCH);
                    break;

                case DRIVE_TO_SWITCH:
                    setSonarTriggerEnabled(true);
                    setRangingEnabled(true);
                    if (scaleRight && startRight || !scaleRight && !startRight)
                    {
                        // Same side, no need to go across.
                        nextState = State.DRIVE_TO_SCALE;
                    }
                    else
                    {
                        // The other side, must go across.
                        // CodeReview: Why fix yourself to lane 3 only? Should let the drive choose.
                        nextState = State.DRIVE_TO_LANE_3;
                    }
                    yDistance = RobotInfo.AUTO_DISTANCE_TO_SWITCH + 24.0 - forwardDriveDistance;
                    if(yDistance <= 0) yDistance = RobotInfo.AUTO_DISTANCE_TO_SWITCH + 24.0;
                    robot.pidDrive.setTarget(0.0, yDistance, robot.targetHeading, false, event);
                    sm.addEvent(event);
                    sm.addEvent(sonarEvent);
                    sm.waitForEvents(nextState); // TODO: Add a timeout to this                        
                    break;

                case DRIVE_TO_LANE_3:
                    setSonarTriggerEnabled(false);
                    sonarDistance = startRight?robot.getLeftSonarDistance():robot.getRightSonarDistance();
                    // Note: distanceFromWall is the distance of the sonar sensor from the field wall.
                    distanceFromWall = RobotInfo.SWITCH_TO_WALL_DISTANCE - sonarDistance - RobotInfo.ROBOT_WIDTH/2.0;
                    robot.tracer.traceInfo(moduleName, "sonarDistance=%.1f, distanceFromWall=%.1f",
                        sonarDistance, distanceFromWall);
                    // CodeReview: the distance should be AUTO_DISTANCE_TO_SWITCH + SWITCH_TO_LANE_3_DISTANCE - currYPos.
                    robot.pidDrive.setTarget(0.0, RobotInfo.ALLIANCE_WALL_TO_LANE_3_DISTANCE - robot.driveBase.getYPosition(), 
                        robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_OPPOSITE_SCALE);
                    break;

                case TURN_TO_OPPOSITE_SCALE:
                    robot.targetHeading = startRight?DRIVE_HEADING_WEST:DRIVE_HEADING_EAST;
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_OPPOSITE_SCALE);
                    break;

                case DRIVE_TO_OPPOSITE_SCALE:
                    yDistance = RobotInfo.FIELD_WIDTH - 2.0*distanceFromWall;
                    robot.pidDrive.setTarget(0.0, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_SCALE);
                    break;

                case TURN_TO_SCALE:
                    robot.targetHeading = DRIVE_HEADING_NORTH;
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.FINISH_DRIVE_TO_SCALE);
                    break;

                case FINISH_DRIVE_TO_SCALE:
                    robot.pidDrive.setTarget(0.0, RobotInfo.ALLIANCE_WALL_TO_SCALE_DISTANCE - RobotInfo.ALLIANCE_WALL_TO_LANE_3_DISTANCE,
                        robot.targetHeading, false, event);
                    // Start raising elevator
                    robot.elevator.setPosition(RobotInfo.ELEVATOR_SCALE_HIGH - 18.0, elevatorEvent, 2.5);
                    sm.waitForSingleEvent(event, State.TURN_TO_FACE_SCALE);
                    break;

                case DRIVE_TO_SCALE:
                    setSonarTriggerEnabled(false);
                    robot.pidDrive.cancel();
                    sonarDistance = startRight?robot.getLeftSonarDistance():robot.getRightSonarDistance();
                    distanceFromWall = RobotInfo.SWITCH_TO_WALL_DISTANCE - sonarDistance - RobotInfo.ROBOT_WIDTH/2.0;
                    xDistance = -(RobotInfo.SCALE_TO_WALL_DISTANCE - RobotInfo.ROBOT_TO_SCALE_DISTANCE - distanceFromWall);
                    robot.tracer.traceInfo(moduleName, "sonarDistance=%.1f, distanceFromWall=%.1f,xDistance=%.1f",
                        sonarDistance, distanceFromWall, xDistance);
                    if(!startRight) xDistance *= -1;
                    yDistance = RobotInfo.FIELD_LENGTH/2.0 - robot.driveBase.getYPosition() - RobotInfo.ROBOT_LENGTH/2.0 - 12.0;
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    // Start raising elevator
                    robot.elevator.setPosition(RobotInfo.ELEVATOR_SCALE_HIGH - 18.0, elevatorEvent, 2.5);
                    sm.waitForSingleEvent(event, State.TURN_TO_FACE_SCALE);
                    break;

                case TURN_TO_FACE_SCALE:
                    robot.targetHeading = startRight?DRIVE_HEADING_WEST:DRIVE_HEADING_EAST;
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.RAISE_ELEVATOR);
                    break;

                case RAISE_ELEVATOR:
                    sm.waitForSingleEvent(elevatorEvent, State.THROW_CUBE);
                    break;

                case THROW_CUBE:
                    // CodeReview: you may want to spit the cube with full power since you are pointing upward.
                    robot.cubePickup.deployPickup();
                    robot.cubePickup.dropCube(RobotInfo.CUBE_PICKUP_DROP_POWER);
                    timer.set(RobotInfo.DROP_CUBE_TIMEOUT, event);
                    sm.waitForSingleEvent(event, State.LOWER_ELEVATOR);
                    break;

                case LOWER_ELEVATOR:
                    robot.cubePickup.stopPickup();
                    robot.elevator.setPosition(RobotInfo.ELEVATOR_MIN_HEIGHT, event, 0.0);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                    done = true;
                    setSonarTriggerEnabled(false);
                    setRangingEnabled(false);
                    sm.stop();
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString());
        }
        return done;
    }
}

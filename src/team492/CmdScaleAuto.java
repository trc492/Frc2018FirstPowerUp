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

public class CmdScaleAuto implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdScaleAuto";
    private static final double[] distances = new double[] { 32.0, 8.0 };

    private enum State
    {
        START,
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
    private TrcEvent event, sonarEvent;
    private TrcStateMachine<State> sm;
    private TrcTimer timer;
    private Position startPosition;
    private boolean startRight;
    private boolean scaleRight;
    private TrcAnalogTrigger<TrcAnalogInput.DataType> sonarTrigger;
    
    private double distanceFromWall = 0;
    private double sonarDistance = 0;
    
    /**
     * 
     * @param robot Robot class
     * @param delay How much to delay (in seconds) before starting
     * @param startPosition Either RobotInfo.LEFT_START_POS, RobotInfo.MID_START_POS, or RobotInfo.RIGHT_START_POS
     */
    public CmdScaleAuto(Robot robot, double delay, double startPosition)
    {
        this.robot = robot;
        this.delay = delay;
        event = new TrcEvent(moduleName);
        sonarEvent = new TrcEvent(moduleName + ".sonarEvent");
        sm = new TrcStateMachine<>(moduleName);
        timer = new TrcTimer(moduleName);

        if(startPosition == RobotInfo.LEFT_START_POS) this.startPosition = Position.LEFT;
        else if(startPosition == RobotInfo.RIGHT_START_POS) this.startPosition = Position.RIGHT;
        else this.startPosition = Position.MIDDLE;

        startRight = this.startPosition == Position.RIGHT;
        scaleRight = robot.gameSpecificMessage.charAt(1) == 'R';

        sonarTrigger = new TrcAnalogTrigger<>(
            "SwitchDistanceTrigger", startRight? robot.leftSonarSensor: robot.rightSonarSensor,
            0, TrcAnalogInput.DataType.INPUT_DATA, distances, this::sonarTriggerEvent);
        
        sm.start(State.START);
    }

    public void setSonarTriggerEnabled(boolean enabled)
    {
        sonarTrigger.setTaskEnabled(enabled);
        if(enabled) sonarEvent.clear();
    }

    private void sonarTriggerEvent(int currZone, int prevZone, double zoneValue)
    {
        robot.tracer.traceInfo("CmdScaleAuto.sonarTriggerEvent", "Sonar event - currZone:%d, prevZone:%d", currZone, prevZone);
        if(currZone == 1 && prevZone == 0)
        {
            sonarEvent.set(true);
        }
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();
        if(done || startPosition == Position.MIDDLE) return true;

        State state = sm.checkReadyAndGetState();
        
        //
        // Print debug info.
        //
        robot.dashboard.displayPrintf(1, "State: %s", state != null? state: sm.getState());
        
        if(state != null)
        {
            double xDistance, yDistance;

            switch(state)
            {
                case START:
                    if(delay != 0.0)
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
                    robot.pidDrive.driveMaintainHeading(0.0, RobotInfo.MOVE_TO_SWITCH_Y_POWER, robot.targetHeading);
                    setSonarTriggerEnabled(true);
                    State nextState;
                    if(scaleRight && startRight || !scaleRight && !startRight)
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
                    sm.waitForSingleEvent(sonarEvent, nextState); // TODO: Add a timeout to this                        
                    break;

                case DRIVE_TO_LANE_3:
                    // CodeReview: What are the below two lines for??? Nobody's using sonarDistance and
                    // distanceFromWall in this state.
                    sonarDistance = startRight?robot.getLeftSonarDistance():robot.getRightSonarDistance();
                    distanceFromWall = RobotInfo.SWITCH_TO_WALL_DISTANCE - sonarDistance;
                    robot.pidDrive.setTarget(0.0, RobotInfo.SWITCH_TO_LANE_3_DISTANCE, 
                        robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_OPPOSITE_SCALE);
                    break;

                case TURN_TO_OPPOSITE_SCALE:
                    robot.targetHeading = startRight?RobotInfo.DRIVE_HEADING_WEST:RobotInfo.DRIVE_HEADING_EAST;
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_OPPOSITE_SCALE);
                    break;

                case DRIVE_TO_OPPOSITE_SCALE:
                    // CodeReview: If you are using distanceFromWall from the previous state, they must be
                    // declared as class variables, not local variables. If not, the values are lost.
                    // Also, don't multiply distanceFromWall by 2. The math below is wrong.
                    // What exactly are you trying to do?
                    yDistance = RobotInfo.FIELD_WIDTH - 2.0*distanceFromWall + RobotInfo.ROBOT_LENGTH;
                    robot.pidDrive.setTarget(0.0, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_SCALE);
                    break;

                case TURN_TO_SCALE:
                    robot.targetHeading = RobotInfo.DRIVE_HEADING_NORTH;
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.FINISH_DRIVE_TO_SCALE);
                    break;

                case FINISH_DRIVE_TO_SCALE:
                    robot.pidDrive.setTarget(0.0, RobotInfo.LANE_3_TO_SCALE_DISTANCE, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_FACE_SCALE);
                    break;

                case DRIVE_TO_SCALE:
                    setSonarTriggerEnabled(false);
                    robot.pidDrive.cancel();
                    sonarDistance = startRight?robot.getLeftSonarDistance():robot.getRightSonarDistance();
                    distanceFromWall = RobotInfo.SWITCH_TO_WALL_DISTANCE - sonarDistance;
                    xDistance = distanceFromWall - RobotInfo.SCALE_TO_WALL_DISTANCE;
                    if(!startRight) xDistance *= -1;
                    yDistance = RobotInfo.SWITCH_TO_SCALE_DISTANCE;
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_FACE_SCALE);
                    break;

                case TURN_TO_FACE_SCALE:
                    robot.targetHeading = startRight?RobotInfo.DRIVE_HEADING_WEST:RobotInfo.DRIVE_HEADING_EAST;
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.RAISE_ELEVATOR);
                    break;

                case RAISE_ELEVATOR:
                    robot.elevator.setPosition(RobotInfo.ELEVATOR_SCALE_HIGH, event, 0.0);
                    sm.waitForSingleEvent(event, State.THROW_CUBE);
                    break;

                case THROW_CUBE:
                    robot.cubePickup.dropCube(RobotInfo.CUBE_PICKUP_DROP_POWER);
                    timer.set(RobotInfo.DROP_CUBE_TIMEOUT, event);
                    sm.waitForSingleEvent(event, State.LOWER_ELEVATOR);
                    break;

                case LOWER_ELEVATOR:
                    robot.elevator.setPosition(RobotInfo.ELEVATOR_MIN_HEIGHT, event, 0.0);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                    done = true;
                    setSonarTriggerEnabled(false);
                    sm.stop();
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString());
        }
        return done;
    }
}

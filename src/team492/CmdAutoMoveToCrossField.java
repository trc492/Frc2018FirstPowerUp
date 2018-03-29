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

import team492.RobotInfo.Position;
import trclib.TrcEvent;
import trclib.TrcStateMachine;
import trclib.TrcTimer;
import trclib.TrcRobot;

public class CmdAutoMoveToCrossField implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoMoveToCrossField";

    private static final double DRIVE_HEADING_EAST = 90.0;
    private static final double DRIVE_HEADING_WEST = -90.0;

    private enum State
    {
        START,
        DRIVE_TO_LANE_3,
        TURN_ACROSS_FIELD,
        DRIVE_ACROSS_FIELD,
        DONE
    }

    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    private Robot robot;
    private double delay;
    private Position startPosition;

    private boolean startRight;

    // CodeReview: Should we give an option to just cross the line and not to lane 3 and cross the field?
    public CmdAutoMoveToCrossField(Robot robot, double delay, Position startPosition)
    {
        robot.tracer.traceInfo(moduleName, "[%.3f] delay=%.1f, startPos=%s",
            Robot.getModeElapsedTime(), delay, startPosition.name());

        this.robot = robot;
        this.delay = delay;
        this.startPosition = startPosition;

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);

        startRight = startPosition == Position.RIGHT_POS;
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

            switch(state)
            {
                case START:
                    if (delay == 0.0)
                    {
                        sm.setState(State.DRIVE_TO_LANE_3);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_TO_LANE_3);
                    }
                    break;

                case DRIVE_TO_LANE_3:
                    xDistance = 0.0;
                    yDistance = RobotInfo.FWD_DISTANCE_3;
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_ACROSS_FIELD);
                    break;

                case TURN_ACROSS_FIELD:
                    xDistance = 0.0;
                    yDistance = 0.0;
                    robot.targetHeading = startRight?DRIVE_HEADING_WEST:DRIVE_HEADING_EAST;
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_ACROSS_FIELD);
                    break;

                case DRIVE_ACROSS_FIELD:
                    xDistance = 0.0;
                    yDistance = RobotInfo.SWITCH_FENCE_WIDTH;
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                    done = true;
                    sm.stop();
                    break;
            }
        }
        return done;
    }
}

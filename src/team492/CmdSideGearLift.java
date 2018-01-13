/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
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
import team492.Robot.Alliance;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdSideGearLift implements TrcRobot.RobotCommand
{
    private static enum State
    {
        DO_DELAY,
        MOVE_FORWARD_ON_SIDE,
        INITIAL_TURN_TOWARDS_AIRSHIP,
        VISION_DEPLOY,
        BACKUP_FROM_AIRSHIP,
        TURN_TOWARDS_LOADING_STATION,
        MOVE_TOWARDS_LOADING_STATION,
        DONE
    }   //enum State

    private static final String moduleName = "CmdSideGearLift";

    private Robot robot;
    private double delay;
    private boolean rightSide;

    private boolean isRed;
    private double sideForwardDistance;
    private double sideLiftAngle;
    private double sideBackupDistance;
    private double sideForwardAngle;
    private double sideLoadingStationAngle;
    private double sideLoadingStationDistance;
    private double sideDiagonalDistance;

    private CmdVisionGearDeploy visionDeploy;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    CmdSideGearLift(Robot robot, double delay, boolean rightSide)
    {
        this.robot = robot;
        this.delay = delay;
        this.rightSide = rightSide;
        this.isRed = robot.alliance == Alliance.RED_ALLIANCE;

        //
        // RobotLength: length of robot (set to RobotInfo.ROBOT_LENGTH: 38 inches).
        // RobotWidth: width of robot (set to RobotInfo.ROBOT_WIDTH: 35 inches).
        // AirshipPanelWidth: width of airship panel (set to RobotInfo.FIELD_AIRSHIP_PANEL_WIDTH: 40 inches).
        // SideLiftAngle: angle to turn for facing side lift (set to RobotInfo.FIELD_SIDELIFT_ANGLE: 60 degrees).
        // AirshipBaseline: distance between driver station wall to baseline of the airship (approx. 114 inches).
        // TargetDistance: distance from front of robot to vision target (set to 36 inches).
        // StartPosition: robot start position from center peg line to side bumper edge of robot.
        // ForwardDistance: initial distance to move forward before turning to find vision target.
        //
        // StartPosition
        //  = (TargetDistance + RobotLength/2 - (AirshipPanelWidth/2)*tan(SideLiftAngle))*sin(SideLiftAngle) +
        //    1.5*AirshipPanelWidth - RobotWidht/2
        //  = (36 + 19 - 20*tan(60))*sin(60) + 60 - 17.5
        //  = 60 inches (approx.)
        //
        // ForwardDistance
        //  = AirshipBaseline -
        //    (TargetDistance + RobotLength/2 - (AirshipPanelWidth/2)*tan(SideLiftAngle))*cos(SideLiftAngle) -
        //    RobotLength/2
        //  = 114 - (36 + 19 - 20*tan(60))*cos(60) - 19
        //  = 85 inches (approx.)
        //
        sideForwardDistance = HalDashboard.getNumber("SideForwardDistance", 85.0);
        sideLiftAngle = Math.abs(HalDashboard.getNumber("SideLiftAngle", 60.0));
        sideBackupDistance = Math.abs(HalDashboard.getNumber("SideBackupDistance", 12.0));
        sideForwardAngle = HalDashboard.getNumber("SideForwardAngle", 0.0);
        sideLoadingStationAngle = Math.abs(HalDashboard.getNumber("SideLoadingStationAngle", 30.0));
        sideLoadingStationDistance = HalDashboard.getNumber("SideLoadingStationDistance", 320.0);
        sideDiagonalDistance = HalDashboard.getNumber("SideDiagonalDistance", 360.0);

        visionDeploy = new CmdVisionGearDeploy(robot);

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);

        robot.tracer.traceInfo(
            moduleName,
            "delay=%.3f, alliance=%s, rightSide=%s, forwardDist=%.1f, liftAngle=%.1f, backupDist=%.1f, lsAngle=%.1f, " +
            "lsDist=%.1f, diagDist=%.1f",
            delay, robot.alliance.toString(), Boolean.toString(rightSide), sideForwardDistance, sideLiftAngle,
            sideBackupDistance, sideLoadingStationAngle, sideLoadingStationDistance, sideDiagonalDistance);
    }   //CmdPidDrive

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
        robot.dashboard.displayPrintf(1, "State: %s", state != null? state.toString(): "Disabled");

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
                        sm.setState(State.MOVE_FORWARD_ON_SIDE);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.MOVE_FORWARD_ON_SIDE);
                    }
                    break;

                case MOVE_FORWARD_ON_SIDE:
                    //
                    // Drive the set distance and heading.
                    //
                    xDistance = 0;
                    yDistance = sideForwardDistance;

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 3.0);
                    sm.waitForSingleEvent(event, State.INITIAL_TURN_TOWARDS_AIRSHIP);
                    break;

                case INITIAL_TURN_TOWARDS_AIRSHIP:
                    //
                    // Turn to face airship.
                    //
                    xDistance = 0;
                    yDistance = 0;
                    robot.targetHeading = rightSide ? -sideLiftAngle : sideLiftAngle;

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 2.0);
                    sm.waitForSingleEvent(event, State.VISION_DEPLOY);
                    break;

                case VISION_DEPLOY:
                    //
                    // Execute visionDeploy to dispense gear on peg
                    //
                    if (visionDeploy.cmdPeriodic(elapsedTime))
                    {
                        sm.setState(State.BACKUP_FROM_AIRSHIP);
                    }
                    break;

                case BACKUP_FROM_AIRSHIP:
                    //
                    // Backup in addition to backup from visionDeploy
                    //
                    xDistance = 0;
                    yDistance = -sideBackupDistance;

//                    robot.encoderXPidCtrl.setOutputRange(-1.0, 1.0);
//                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 1.0);
                    sm.waitForSingleEvent(event, State.TURN_TOWARDS_LOADING_STATION);
                    break;

                case TURN_TOWARDS_LOADING_STATION:
                    //
                    // Turn towards loading station after backing up from airship
                    //
                    xDistance = 0;
                    yDistance = 0;
                    robot.targetHeading = isRed && !rightSide || !isRed && rightSide? sideForwardAngle:
                                          isRed? -sideLoadingStationAngle: sideLoadingStationAngle;

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 2.0);
                    sm.waitForSingleEvent(event, State.MOVE_TOWARDS_LOADING_STATION);
                    break;

                case MOVE_TOWARDS_LOADING_STATION:
                    //
                    // Move towards (but not in) loading station
                    //
                    xDistance = 0.0;
                    yDistance =
                        isRed && !rightSide || !isRed && rightSide? sideLoadingStationDistance: sideDiagonalDistance;

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
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
    }   //cmdPeriodic

}   //class CmdSideGearLift

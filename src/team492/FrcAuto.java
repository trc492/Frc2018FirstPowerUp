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

import java.util.Date;

import common.CmdPidDrive;
import common.CmdTimedDrive;
import frclib.FrcChoiceMenu;
import hallib.HalDashboard;
import trclib.TrcRobot;

public class FrcAuto implements TrcRobot.RobotMode
{
    public static enum AutoStrategy
    {
        // Different choices for autonomous
        POWER_UP_AUTO,
        SCALE_AUTO,
        X_TIMED_DRIVE,
        Y_TIMED_DRIVE,
        X_DISTANCE_DRIVE,
        Y_DISTANCE_DRIVE,
        TURN_DEGREES,
        DO_NOTHING
    } // enum AutoStrategy

    public static enum ForwardDistance
    {
        // Different choices for forward distances
        FWD_DISTANCE_1,
        FWD_DISTANCE_2,
        FWD_DISTANCE_3,
        CUSTOM
    } // enum ForwardDistance

    public static enum Approach
    {
        // Different choices for approaches
        SIDE,
        FRONT
    } // enum Approach

    public static enum StartPosition
    {
        // Different choices for start positions
        LEFT_START_POS,
        MID_START_POS,
        RIGHT_START_POS
    } // enum StartPosition

    private Robot robot;

    //
    // Menus.
    //
    private FrcChoiceMenu<AutoStrategy> autoStrategyMenu;
    private FrcChoiceMenu<ForwardDistance> forwardDistanceMenu;
    private FrcChoiceMenu<Approach> approachMenu;
    private FrcChoiceMenu<StartPosition> startPositionMenu;

    private AutoStrategy autoStrategy;
    private ForwardDistance forwardDistance;
    private StartPosition startPosition;
    private double delay;
    private boolean flipInFlight;

    private double forwardDriveDistance;
    private boolean sideApproach;
    private double robotStartPosition;

    private TrcRobot.RobotCommand autoCommand;

    public FrcAuto(Robot robot)
    {
        this.robot = robot;
        //
        // Create Autonomous Mode specific menus.
        //
        autoStrategyMenu = new FrcChoiceMenu<>("Autonomous Strategies");
        forwardDistanceMenu = new FrcChoiceMenu<>("Forward Distances");
        approachMenu = new FrcChoiceMenu<>("Approaches");
        startPositionMenu = new FrcChoiceMenu<>("Start Positions");

        //
        // Populate Autonomous Mode menus.
        //
        autoStrategyMenu.addChoice("Power Up Auto", AutoStrategy.POWER_UP_AUTO, true, false);
        autoStrategyMenu.addChoice("Scale Auto", AutoStrategy.SCALE_AUTO, false, false);
        autoStrategyMenu.addChoice("X Timed Drive", AutoStrategy.X_TIMED_DRIVE, false, false);
        autoStrategyMenu.addChoice("Y Timed Drive", AutoStrategy.Y_TIMED_DRIVE, false, false);
        autoStrategyMenu.addChoice("X Distance Drive", AutoStrategy.X_DISTANCE_DRIVE, false, false);
        autoStrategyMenu.addChoice("Y Distance Drive", AutoStrategy.Y_DISTANCE_DRIVE, false, false);
        autoStrategyMenu.addChoice("Turn Degrees", AutoStrategy.TURN_DEGREES, false, false);
        autoStrategyMenu.addChoice("Do Nothing", AutoStrategy.DO_NOTHING, false, true);

        forwardDistanceMenu.addChoice("Distance 1", ForwardDistance.FWD_DISTANCE_1, false, false);
        forwardDistanceMenu.addChoice("Distance 2", ForwardDistance.FWD_DISTANCE_2, true, false);
        forwardDistanceMenu.addChoice("Distance 3", ForwardDistance.FWD_DISTANCE_3, false, false);
        forwardDistanceMenu.addChoice("Custom Distance", ForwardDistance.CUSTOM, false, true);

        approachMenu.addChoice("Front Approach", Approach.FRONT, false, false);
        approachMenu.addChoice("Side Approach", Approach.SIDE, true, true);

        startPositionMenu.addChoice("Left Side Start", StartPosition.LEFT_START_POS, true, false);
        startPositionMenu.addChoice("Middle Start", StartPosition.MID_START_POS, false, false);
        startPositionMenu.addChoice("Right Side Start", StartPosition.RIGHT_START_POS, false, true);
    } // FrcAuto

    //
    // Implements TrcRobot.RunMode.
    //

    @Override
    public void startMode()
    {
        robot.encoderYPidCtrl.setOutputLimit(0.6);
        robot.encoderXPidCtrl.setOutputLimit(RobotInfo.DRIVE_MAX_XPID_POWER);
        
        HalDashboard.getInstance().clearDisplay();

        if (Robot.USE_TRACELOG)
            robot.startTraceLog(null);

        Date now = new Date();
        robot.tracer.traceInfo(Robot.programName, "%s: ***** Starting autonomous *****", now.toString());
        robot.tracer.traceInfo(Robot.programName, "%s_%s_%3d (%s%d) [FMSConnected=%b]", robot.eventName,
            robot.matchType.toString(), robot.matchNumber, robot.alliance.toString(), robot.location,
            robot.ds.isFMSAttached());
        //
        // Retrieve menu choice values.
        //
        forwardDistance = forwardDistanceMenu.getCurrentChoiceObject();
        switch (forwardDistance)
        {
            case FWD_DISTANCE_1:
            default:
                forwardDriveDistance = RobotInfo.FWD_DISTANCE_1;
                break;

            case FWD_DISTANCE_2:
                forwardDriveDistance = RobotInfo.FWD_DISTANCE_2;
                break;

            case FWD_DISTANCE_3:
                forwardDriveDistance = RobotInfo.FWD_DISTANCE_3;
                break;

            case CUSTOM:
                forwardDriveDistance = -1.0;
                break;
        }

        sideApproach = approachMenu.getCurrentChoiceObject() == Approach.SIDE;

        startPosition = startPositionMenu.getCurrentChoiceObject();
        switch (startPosition)
        {
            case LEFT_START_POS:
                robotStartPosition = RobotInfo.LEFT_START_POS;
                break;
            case MID_START_POS:
                robotStartPosition = RobotInfo.MID_START_POS;
                break;
            case RIGHT_START_POS:
                robotStartPosition = RobotInfo.RIGHT_START_POS;
                break;
        }

        autoStrategy = autoStrategyMenu.getCurrentChoiceObject();
        delay = HalDashboard.getNumber("Delay", 0.0);
        flipInFlight = HalDashboard.getBoolean("FlipInFlight", false);

        switch (autoStrategy)
        {
            case POWER_UP_AUTO:
                autoCommand = new CmdPowerUpAuto(
                    robot, delay, forwardDriveDistance, sideApproach, robotStartPosition, flipInFlight);
                break;

            case SCALE_AUTO:
            	autoCommand = new CmdScaleAuto(robot, delay, robotStartPosition);
            	break;

            case X_TIMED_DRIVE:
                autoCommand = new CmdTimedDrive(robot, delay, robot.driveTime, robot.drivePower, 0.0, 0.0);
                break;

            case Y_TIMED_DRIVE:
                autoCommand = new CmdTimedDrive(robot, delay, robot.driveTime, 0.0, robot.drivePower, 0.0);
                break;

            case X_DISTANCE_DRIVE:
                autoCommand = new CmdPidDrive(
                    robot, robot.pidDrive, robot.encoderXPidCtrl, robot.encoderYPidCtrl, robot.gyroTurnPidCtrl,
                    delay, robot.driveDistance, 0.0, 0.0, robot.drivePowerLimit, false);
                break;

            case Y_DISTANCE_DRIVE:
                autoCommand = new CmdPidDrive(
                    robot, robot.pidDrive, robot.encoderXPidCtrl, robot.encoderYPidCtrl, robot.gyroTurnPidCtrl,
                    delay, 0.0, robot.driveDistance, 0.0, robot.drivePowerLimit, false);
                break;

            case TURN_DEGREES:
                autoCommand = new CmdPidDrive(
                    robot, robot.pidDrive, robot.encoderXPidCtrl, robot.encoderYPidCtrl, robot.gyroTurnPidCtrl,
                    delay, 0.0, 0.0, robot.turnDegrees, robot.drivePowerLimit, false);
                break;

            default:
            case DO_NOTHING:
                autoCommand = null;
                break;
        }

        robot.setVisionEnabled(true);
        robot.driveBase.resetPosition();
        robot.targetHeading = 0.0;
    } // startMode

    @Override
    public void stopMode()
    {
        robot.setVisionEnabled(false);

        if (Robot.USE_TRACELOG)
            robot.stopTraceLog();
    } // stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        robot.updateDashboard();
        robot.announceSafety();
    } // runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        if (autoCommand != null)
        {
            autoCommand.cmdPeriodic(elapsedTime);

            if (robot.pidDrive.isActive())
            {
                robot.encoderXPidCtrl.printPidInfo(robot.tracer, elapsedTime, robot.battery);
                robot.encoderYPidCtrl.printPidInfo(robot.tracer, elapsedTime, robot.battery);
                robot.gyroTurnPidCtrl.printPidInfo(robot.tracer, elapsedTime, robot.battery);
            }
        }
    } // runContinuous

} // class FrcAuto

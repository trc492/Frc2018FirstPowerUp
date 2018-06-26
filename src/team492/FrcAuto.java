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

import common.CmdPidDrive;
import common.CmdTimedDrive;
import frclib.FrcChoiceMenu;
import hallib.HalDashboard;
import team492.RobotInfo.Position;
import trclib.TrcRobot;
import trclib.TrcRobot.RunMode;
import trclib.TrcTaskMgr;

public class FrcAuto implements TrcRobot.RobotMode
{
    private static final String moduleName = "FrcAuto";
    private static final boolean DO_UPDATES = false;

    public static enum AutoStrategy
    {
        // Different choices for autonomous
        MOTION_PROFILE_TEST,
        X_TIMED_DRIVE,
        Y_TIMED_DRIVE,
        X_DISTANCE_DRIVE,
        Y_DISTANCE_DRIVE,
        TURN_DEGREES,
        DO_NOTHING
    } // enum AutoStrategy

    public static enum YesOrNo
    {
        YES,
        NO
    } // enum YesOrNo

    public static enum Lane
    {
        // Different choices for crossing lanes
        LANE1,
        LANE2,
        LANE3,
        CUSTOM
    } // enum Lane
    
    public static enum ScaleOrSwitch
    {
        SCALE,
        SWITCH
    }

    private Robot robot;

    //
    // Menus.
    //
    private FrcChoiceMenu<AutoStrategy> autoStrategyMenu;

    private AutoStrategy autoStrategy;

    private double delay;
    private TrcRobot.RobotCommand autoCommand;

    public FrcAuto(Robot robot)
    {
        this.robot = robot;
        //
        // Create Autonomous Mode specific menus.
        //
        autoStrategyMenu = new FrcChoiceMenu<>("Auto/AutoStrategies");

        //
        // Populate Autonomous Mode menus.
        //
        //
        autoStrategyMenu.addChoice("Motion Profile", AutoStrategy.MOTION_PROFILE_TEST, true, false);
        autoStrategyMenu.addChoice("X Timed Drive", AutoStrategy.X_TIMED_DRIVE, false, false);
        autoStrategyMenu.addChoice("Y Timed Drive", AutoStrategy.Y_TIMED_DRIVE, false, false);
        autoStrategyMenu.addChoice("X Distance Drive", AutoStrategy.X_DISTANCE_DRIVE, false, false);
        autoStrategyMenu.addChoice("Y Distance Drive", AutoStrategy.Y_DISTANCE_DRIVE, false, false);
        autoStrategyMenu.addChoice("Turn Degrees", AutoStrategy.TURN_DEGREES, false, false);
        autoStrategyMenu.addChoice("Do Nothing", AutoStrategy.DO_NOTHING, false, true);
    } // FrcAuto

    //
    // Implements TrcRobot.RunMode.
    //

    @Override
    public void startMode(RunMode prevMode)
    {
        final String funcName = moduleName + ".startMode";

        robot.getGameInfo();
        robot.globalTracer.traceInfo(funcName, "%s_%s%03d (%s%d) [FMSConnected=%b] msg=%s",
            robot.eventName, robot.matchType, robot.matchNumber, robot.alliance.toString(), robot.location,
            robot.ds.isFMSAttached(), robot.gameSpecificMessage);

        robot.encoderYPidCtrl.setOutputLimit(0.6);  //CodeReview: can we use RobotInfo.DRIVE_MAX_YPID_POWER?
        robot.encoderXPidCtrl.setOutputLimit(RobotInfo.DRIVE_MAX_XPID_POWER);

        //
        // Retrieve menu choice values.
        //
        autoStrategy = autoStrategyMenu.getCurrentChoiceObject();

        delay = HalDashboard.getNumber("Auto/Delay", 0.0);

        switch (autoStrategy)
        {
            case MOTION_PROFILE_TEST:
                MotionProfileTest test = new MotionProfileTest("MP", robot);
                test.start();
                autoCommand = test;
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

            case DO_NOTHING:
                autoCommand = null;
                break;
        }
    } // startMode

    @Override
    public void stopMode(RunMode nextMode)
    {
        TrcTaskMgr.getInstance().printTaskPerformanceMetrics(robot.globalTracer);
    } // stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        if (DO_UPDATES)
        {
            robot.updateDashboard(RunMode.AUTO_MODE);
            robot.announceSafety();
            robot.diagnostics.updateDiagnosticsAndDashboard();
        }
    } // runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        final String funcName = moduleName + ".runContinuous";

        if (autoCommand != null)
        {
            autoCommand.cmdPeriodic(elapsedTime);

            if (robot.pidDrive.isActive())
            {
                robot.encoderXPidCtrl.printPidInfo(robot.globalTracer, elapsedTime, robot.battery);
                robot.encoderYPidCtrl.printPidInfo(robot.globalTracer, elapsedTime, robot.battery);
                robot.gyroTurnPidCtrl.printPidInfo(robot.globalTracer, elapsedTime, robot.battery);
            }

            if (robot.elevator.elevator.isActive())
            {
                robot.elevator.elevatorPidCtrl.printPidInfo(robot.globalTracer, elapsedTime, robot.battery);
                robot.globalTracer.traceInfo(funcName, "Elevator limit switch: %b/%b",
                    robot.elevator.elevatorMotor.isLowerLimitSwitchActive(),
                    robot.elevator.elevatorMotor.isUpperLimitSwitchActive());
            }
        }
    } // runContinuous

} // class FrcAuto

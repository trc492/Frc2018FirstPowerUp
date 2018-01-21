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
import team492.PixyVision.TargetInfo;
import trclib.TrcEvent;
import trclib.TrcPidController.PidCoefficients;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdVisionGearDeploy implements TrcRobot.RobotCommand
{
    private static enum State
    {
        TURN_TO_TARGET,
        DRIVE_TOWARDS_TARGET,
        DEPLOY_GEAR,
        BACKUP,
        DONE
    }   //enum State

    private static final String moduleName = "CmdVisionGearDeploy";

    private Robot robot;

    private double visionTargetDistance;
    private double visionGearDeployTime;
    private double visionBackupDistance;

    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    CmdVisionGearDeploy(Robot robot)
    {
        this.robot = robot;

        visionTargetDistance = HalDashboard.getNumber("VisionTargetDistance", 7.0);
        visionGearDeployTime = HalDashboard.getNumber("VisionGearDeployTime", 0.3);
        visionBackupDistance = HalDashboard.getNumber("VisionBackupDistance", 36.0);

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        start();

        robot.tracer.traceInfo(
            moduleName, "dist=%.1f, deployTime=%.1f, backupDist=%.1f",
            visionTargetDistance, visionGearDeployTime, visionBackupDistance);
    }   //CmdVisionGearDeploy

    public void start()
    {
        sm.start(State.DRIVE_TOWARDS_TARGET);
    }   //start

    public void stop()
    {
        sm.stop();
        robot.pidDrive.cancel();
        robot.visionPidDrive.cancel();
    }   //start

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
        TargetInfo targetInfo;
        State state = sm.getState();
        robot.dashboard.displayPrintf(1, "State: %s", state != null ? state.toString() : "Disabled");

        if (sm.isReady())
        {
            double xDistance, yDistance;
            state = sm.getState();

            switch (state)
            {
                case TURN_TO_TARGET:
                    targetInfo = robot.pixy.getTargetInfo();
                    double angle = targetInfo != null? targetInfo.angle: 0.0;
                    xDistance = yDistance = 0.0;
                    robot.targetHeading += angle;

                    if (angle != 0.0 && Math.abs(angle) < RobotInfo.GYRO_TURN_SMALL_THRESHOLD)
                    {
                        robot.gyroTurnPidCtrl.setPidCoefficients(
                            new PidCoefficients(
                                RobotInfo.GYRO_TURN_SMALL_KP,
                                RobotInfo.GYRO_TURN_SMALL_KI,
                                RobotInfo.GYRO_TURN_SMALL_KD,
                                0.0));
                    }
                    robot.tracer.traceInfo(
                        moduleName, "Target Info: %s", targetInfo != null? targetInfo.toString(): "not found");

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 1.0);
                    sm.waitForSingleEvent(event, State.DRIVE_TOWARDS_TARGET);
                    break;

                case DRIVE_TOWARDS_TARGET:
                    robot.gyroTurnPidCtrl.setPidCoefficients(
                        new PidCoefficients(
                            RobotInfo.GYRO_TURN_KP, RobotInfo.GYRO_TURN_KI, RobotInfo.GYRO_TURN_KD, 0.0));
                    robot.visionTurnPidCtrl.setNoOscillation(true);
                    robot.sonarDrivePidCtrl.setNoOscillation(true);
                    robot.encoderYPidCtrl.setNoOscillation(true);
                    robot.gyroTurnPidCtrl.setNoOscillation(true);
                    xDistance = 0.0;
                    yDistance = visionTargetDistance;
                    double heading = 0.0;

                    robot.visionPidDrive.setTarget(xDistance, yDistance, heading, false, event, 2.0);
                    sm.waitForSingleEvent(event, State.DEPLOY_GEAR);
                    break;

                case DEPLOY_GEAR:
                    //
                    // Place gear on peg.
                    //
                    timer.set(visionGearDeployTime, event);
                    sm.waitForSingleEvent(event, State.BACKUP);
                    break;

                case BACKUP:
                    //
                    // Backup from the lift.
                    //
                    xDistance = 0.0;
                    yDistance = -visionBackupDistance;
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event, 1.5);
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

}   //class CmdVisionGearDeploy

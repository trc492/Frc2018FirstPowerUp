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

import common.CmdTimedDrive;
import trclib.TrcEvent;
import trclib.TrcRevBlinkin.LEDPattern;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class CmdAutoDiagnostics implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoDiagnostics";
    private static final double DRIVE_TIME = 5.0;
    private static final double ENCODER_Y_ERROR_THRESHOLD = 2.0/RobotInfo.ENCODER_Y_INCHES_PER_COUNT; // 2 inches
    private static final double FLIPPER_EXTEND_DELAY = 3.0;
    private static final double ELEVATOR_ERROR_THRESHOLD = 2.0; // 2 inches
    private static final double GRABBER_DELAY = 3.0;
    
    private enum State
    {
        START,
        SPIN_MOTORS_FORWARD,
        SPIN_MOTORS_BACKWARD,
        TEST_FLIPPERS,
        RAISE_ELEVATOR,
        CHECK_ELEVATOR,
        TOGGLE_GRABBER,
        TOGGLE_GRABBER_AGAIN,
        DONE
    }
    
    private Robot robot;
    private TrcStateMachine<State> sm;
    private TrcEvent event;
    private TrcTimer timer;
    private CmdTimedDrive timedDrive;
    public CmdAutoDiagnostics(Robot robot)
    {
        this.robot = robot;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        timer = new TrcTimer(moduleName);
        
        sm.start(State.START);
    }
    
    private CmdTimedDrive createTimedDrive(double xPower, double yPower)
    {
        return new CmdTimedDrive(robot, 0.0, DRIVE_TIME, xPower, yPower, 0.0);
    }
    
    private double[] getMotorPositions()
    {
        double[] motorPositions = new double[]
            {
                    robot.leftFrontWheel.getPosition(),
                    robot.rightFrontWheel.getPosition(),
                    robot.leftRearWheel.getPosition(),
                    robot.rightRearWheel.getPosition()
            };
        return motorPositions;
    }
    
    private String getMotorName(int motorIndex, int numMotors)
    {
        String motorName = null;
        if(numMotors == 4)
        {
            switch(motorIndex)
            {
                case 0:
                    motorName = "lf";
                    break;

                case 1:
                    motorName = "rf";
                    break;

                case 2:
                    motorName = "lr";
                    break;

                case 3:
                    motorName = "rr";
                    break;
            }
        }
        return motorName;
    }
    
    private void checkMotorEncoders()
    {
        double[] motorPositions = getMotorPositions();
        double avgPos = 0;
        for(double pos:motorPositions)
        {
            avgPos += pos;
        }
        avgPos /= motorPositions.length;
        
        for(int i = 0; i < motorPositions.length; i++)
        {
            String motorName = getMotorName(i, motorPositions.length);
            double error = avgPos - motorPositions[i];
            if(Math.abs(error) >= ENCODER_Y_ERROR_THRESHOLD)
            {
                robot.tracer.traceWarn(moduleName,
                    "Motor %s may not be working! AvgPos:%.2f, Pos:%.2f, Error:%.2f",
                    motorName, avgPos, motorPositions[i], error);
            } else
            {
                robot.tracer.traceInfo(moduleName,
                    "Motor %s seems to be working! AvgPos:%.2f, Pos:%.2f, Error:%.2f",
                    motorName, avgPos, motorPositions[i], error);
            }
        }
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();
        if (done) return true;

        State state = sm.checkReadyAndGetState();

        //
        // Print debug info.
        //
        robot.dashboard.displayPrintf(1, "State: %s", state != null? state: sm.getState());
        
        double yPosition;
        if(state != null)
        {
            switch(state)
            {
                case START:
                    robot.ledStrip.setPattern(LEDPattern.FixedStrobeRed);
                    timedDrive = null;
                    sm.setState(State.SPIN_MOTORS_FORWARD);
                    break;
                    
                case SPIN_MOTORS_FORWARD:
                    if(timedDrive == null)
                    {
                        robot.driveBase.resetPosition();
                        timedDrive = createTimedDrive(0.0,0.7);
                    }
                    
                    if(timedDrive.cmdPeriodic(elapsedTime))
                    {
                        checkMotorEncoders();
                        yPosition = robot.driveBase.getYPosition();
                        if(yPosition > 0)
                        {
                            robot.tracer.traceInfo(moduleName, "Spun forward! Motors working okay! yPosition=%.2f", yPosition);
                        }
                        else
                        {
                            robot.tracer.traceInfo(moduleName, "Tried to spin forward! Motors are not okay! yPosition=%.2f", yPosition);
                        }
                        
                        timedDrive = null;
                        sm.setState(State.SPIN_MOTORS_BACKWARD);
                    }
                    break;
                    
                case SPIN_MOTORS_BACKWARD:
                    if(timedDrive == null)
                    {
                        robot.driveBase.resetPosition();
                        timedDrive = createTimedDrive(0.0,-0.7);
                    }
                    
                    if(timedDrive.cmdPeriodic(elapsedTime))
                    {
                        checkMotorEncoders();
                        yPosition = robot.driveBase.getYPosition();
                        if(yPosition < 0)
                        {
                            robot.tracer.traceInfo(moduleName, "Spun backward! Motors working okay, probably! yPosition=%.2f", yPosition);
                        }
                        else
                        {
                            robot.tracer.traceInfo(moduleName, "Tried to spin backward! Motors are not okay, probably! yPosition=%.2f", yPosition);
                        }
                        sm.setState(State.TEST_FLIPPERS);
                    }
                    break;
                    
                case TEST_FLIPPERS:
                    robot.tracer.traceInfo(moduleName, "Attempting to extend flippers for %.1f seconds!", FLIPPER_EXTEND_DELAY);
                    robot.leftFlipper.timedExtend(FLIPPER_EXTEND_DELAY);
                    robot.rightFlipper.timedExtend(FLIPPER_EXTEND_DELAY);
                    timer.set(FLIPPER_EXTEND_DELAY, event);
                    sm.waitForSingleEvent(event, State.RAISE_ELEVATOR);
                    break;
                    
                case RAISE_ELEVATOR:
                    robot.tracer.traceInfo(moduleName, "Attempting to raise elevator to %.1f inches!", RobotInfo.ELEVATOR_MAX_HEIGHT);
                    robot.elevator.setPosition(RobotInfo.ELEVATOR_MAX_HEIGHT, event, 0.0);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;
                    
                case CHECK_ELEVATOR:
                    double elevatorHeight = robot.elevator.getPosition();
                    double error = RobotInfo.ELEVATOR_MAX_HEIGHT - elevatorHeight;
                    if(error >= ELEVATOR_ERROR_THRESHOLD)
                    {
                        robot.tracer.traceWarn(moduleName,
                            "Elevator innacurate! Target: %.2f, Actual: %.2f, Error: %.2f",
                            RobotInfo.ELEVATOR_MAX_HEIGHT, elevatorHeight, error);
                    } else
                    {
                        robot.tracer.traceInfo(moduleName, "Elevator working fine!, Target: %.2f, Actual: %.2f, Error: %.2f",
                            RobotInfo.ELEVATOR_MAX_HEIGHT, elevatorHeight, error);
                    }
                    robot.elevator.setPosition(RobotInfo.ELEVATOR_MIN_HEIGHT, event, 0.0);
                    sm.waitForSingleEvent(event, State.TOGGLE_GRABBER);
                    break;
                    
                case TOGGLE_GRABBER:
                    if(robot.cubePickup.isClawOpen())
                    {
                        robot.cubePickup.closeClaw();
                    } else
                    {
                        robot.cubePickup.openClaw();
                    }
                    
                    if(robot.cubePickup.isPickupDeployed())
                    {
                        robot.cubePickup.raisePickup();
                    } else
                    {
                        robot.cubePickup.deployPickup();
                    }
                    robot.cubePickup.setPickupPower(1.0);
                    timer.set(GRABBER_DELAY, event);
                    sm.waitForSingleEvent(event, State.TOGGLE_GRABBER_AGAIN);
                    break;
                    
                case TOGGLE_GRABBER_AGAIN:
                    if(robot.cubePickup.isClawOpen())
                    {
                        robot.cubePickup.closeClaw();
                    } else
                    {
                        robot.cubePickup.openClaw();
                    }
                    
                    if(robot.cubePickup.isPickupDeployed())
                    {
                        robot.cubePickup.raisePickup();
                    } else
                    {
                        robot.cubePickup.deployPickup();
                    }
                    robot.cubePickup.dropCube(1.0);
                    timer.set(GRABBER_DELAY, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;
                    
                case DONE:
                    done = true;
                    break;
            }
        }
        
        return done;
    }

}

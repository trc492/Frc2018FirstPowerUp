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

package frclib;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import trclib.TrcPidController.PidCoefficients;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;

import java.security.InvalidParameterException;

/**
 * This is a super sketchy implementation of motion profiling. It streams the points to the buffer, and then executes
 * it. Also, the points are processed 2x as fast as the first point.
 * This was written by using these resources:
 * https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/Java/MotionProfile/src/org/usfirst/frc/team217/robot
 * https://github.com/CrossTheRoadElec/Phoenix-Documentation/blob/master/Talon%20SRX%20Motion%20Profile%20Reference%20Manual.pdf
 */

public class FrcSketchyMotionProfile
{
    private static final int MAX_POINT_BUFFER_SIZE = 2048; // Max number of points talon buffer can hold
    private static final int PERIODIC_BUFFER_FILL_SIZE = 512; // Number of points to add to buffer periodically
    private static final int MIN_POINTS_IN_TALON = 10;
    private static final TrajectoryDuration DEFAULT_TRAJECTORY_DURATION = TrajectoryDuration.Trajectory_Duration_10ms;

    private enum State
    {
        START, WAIT_FOR_POINTS, MONITOR_PATH, DONE
    }

    private PidCoefficients pidCoefficients;
    private int pidSlot;
    private double inchesPerEncoderTick;
    private FrcCANTalon leftMaster, rightMaster;
    private TrcTaskMgr.TaskObject taskObject;
    private double[][][] points;
    private TrcStateMachine<State> sm;
    private int fillIndex = 0;
    private boolean filled = false;
    private Notifier notifier;
    private MotionProfileStatus[] statuses;
    private boolean cancelled = false;

    public FrcSketchyMotionProfile(String instanceName, PidCoefficients pidCoefficients, int pidSlot, double inchesPerEncoderTick)
    {
        this.pidCoefficients = pidCoefficients;
        this.pidSlot = pidSlot;
        this.inchesPerEncoderTick = inchesPerEncoderTick;

        sm = new TrcStateMachine<>(instanceName);
        notifier = new Notifier(this::processPointBuffer);

        taskObject = TrcTaskMgr.getInstance().createTask(instanceName + ".postContinuousTask", this::postContinuousTask);
    }

    public void setLeftMotors(FrcCANTalon...leftMotors)
    {
        if(leftMotors.length == 0)
        {
            throw new InvalidParameterException("Cannot pass empty array of motors!");
        }

        this.leftMaster = leftMotors[0];

        leftMaster.motor.config_kP(pidSlot, pidCoefficients.kP, 10);
        leftMaster.motor.config_kI(pidSlot, pidCoefficients.kI, 10);
        leftMaster.motor.config_kD(pidSlot, pidCoefficients.kD, 10);
        leftMaster.motor.config_kF(pidSlot, pidCoefficients.kF, 10);

        leftMaster.motor.changeMotionControlFramePeriod(5);

        if(leftMotors.length > 1)
        {
            for(int i = 1; i < leftMotors.length; i++)
            {
                leftMotors[i].motor.set(ControlMode.Follower, leftMaster.motor.getDeviceID());
            }
        }
    }

    public void setRightMotors(FrcCANTalon...rightMotors)
    {
        if(rightMotors.length == 0)
        {
            throw new InvalidParameterException("Cannot pass empty array of motors!");
        }

        this.rightMaster = rightMotors[0];

        rightMaster.motor.config_kP(pidSlot, pidCoefficients.kP, 10);
        rightMaster.motor.config_kI(pidSlot, pidCoefficients.kI, 10);
        rightMaster.motor.config_kD(pidSlot, pidCoefficients.kD, 10);
        rightMaster.motor.config_kF(pidSlot, pidCoefficients.kF, 10);

        rightMaster.motor.changeMotionControlFramePeriod(5);

        if(rightMotors.length > 1)
        {
            for(int i = 1; i < rightMotors.length; i++)
            {
                rightMotors[i].motor.set(ControlMode.Follower, rightMaster.motor.getDeviceID());
            }
        }
    }

    /**
     * Follow the supplied path using motion profiling.
     * @param points 2d array of points of dimension [numPoints, 2, 3].
     *               In the 2nd dimension, index 0 is left motors, index 1 is right motors.
     *               In the 3rd dimension, index 0 is position, index 1 is velocity, and index 2 is timestep
     */
    public void start(double[][][] points)
    {
        if(points[0].length != 2 || points[0][0].length != 3)
        {
            throw new IllegalArgumentException("path must be 3d array with size [numPoints, 2, 3]!");
        }

        if(leftMaster == null || rightMaster == null)
        {
            throw new IllegalStateException("Left and right motors must be set before calling start()!");
        }

        statuses = new MotionProfileStatus[]{new MotionProfileStatus(), new MotionProfileStatus()};

        this.points = new double[points.length][points[0].length][points[1].length];
        for(int i = 0; i < points.length; i++)
        {
            for(int j = 0; j < 2; j++)
            {
                this.points[i][j][0] = points[i][j][0] / inchesPerEncoderTick;
                this.points[i][j][1] = points[i][j][1] / inchesPerEncoderTick;
                this.points[i][j][2] = points[i][j][2];
            }
        }

        double duration = points[0][0][0];
        double updatePeriod = duration/2; // 2x as fast as trajectory duration
        notifier.startPeriodic(updatePeriod/1000d); // Convert from milliseconds to seconds

        setTaskEnabled(true);
        leftMaster.motor.changeMotionControlFramePeriod((int)updatePeriod);
        rightMaster.motor.changeMotionControlFramePeriod((int)updatePeriod);
    }

    public boolean isActive()
    {
        return sm.isEnabled();
    }

    public boolean isCancelled()
    {
        return cancelled;
    }

    public void cancel()
    {
        cancelled = true;
        stop();
    }

    private void stop()
    {
        notifier.stop();
        sm.stop();
        setTaskEnabled(false);
        setTalonValue(SetValueMotionProfile.Disable);
    }

    private void setTaskEnabled(boolean enabled)
    {
        if(enabled)
        {
            taskObject.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            taskObject.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }

    private void postContinuousTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        if(!sm.isEnabled()) return;

        State state = sm.getState();
        fillStatuses();

        switch(state){
            case START:
                // Fill the top buffer. If points.length < MAX_POINT_BUFFER_SIZE, fill completely.
                filled = false;
                cancelled = false;
                fillPointBuffer();
                sm.setState(State.WAIT_FOR_POINTS);
                break;

            case WAIT_FOR_POINTS:
                // Wait for the bottom buffer to get enough trajectory points
                if(hasEnoughPoints())
                {
                    sm.setState(State.MONITOR_PATH);
                }
                break;

            case MONITOR_PATH:
                fillPointBuffer(); // Keep filling points into top buffer. (only useful if points.length > MAX_POINT_BUFFER_SIZE)
                setTalonValue(SetValueMotionProfile.Enable); // Keep sending the enable signal
                if(isDone())
                {
                    sm.setState(State.DONE);
                }
                break;

            case DONE:
                stop();
                break;
        }
    }

    private boolean isDone()
    {
        for(MotionProfileStatus status:statuses)
        {
            if(!status.activePointValid || !status.isLast) return false;
        }
        return true;
    }

    private void setTalonValue(SetValueMotionProfile value)
    {
        if(leftMaster != null) leftMaster.motor.set(ControlMode.MotionProfile, value.value);
        if(rightMaster != null) rightMaster.motor.set(ControlMode.MotionProfile, value.value);
    }

    private void fillStatuses()
    {
        leftMaster.motor.getMotionProfileStatus(statuses[0]);
        rightMaster.motor.getMotionProfileStatus(statuses[1]);
    }

    private boolean hasEnoughPoints()
    {
        for(MotionProfileStatus status:statuses)
        {
            if(status.btmBufferCnt < MIN_POINTS_IN_TALON) return false;
        }
        return true;
    }

    private boolean hasBufferRoom()
    {
        for(MotionProfileStatus status:statuses)
        {
            if(status.topBufferRem < PERIODIC_BUFFER_FILL_SIZE) return false;
        }
        return true;
    }

    private void processPointBuffer()
    {
        leftMaster.motor.processMotionProfileBuffer();
        rightMaster.motor.processMotionProfileBuffer();
    }
    
    /**
     * Get a TrajectoryDuration object with the specified time
     * @param duration Milliseconds for the duration
     * @return TrajectoryDuration object representing the supplied time, if available
     */
    private TrajectoryDuration getTrajectoryDuration(int duration)
    {
        TrajectoryDuration dur = TrajectoryDuration.Trajectory_Duration_0ms;
        dur = dur.valueOf(duration);
        
        if(dur.value != duration)
        {
            DriverStation.reportError("Duration not supported!", false);
            dur = DEFAULT_TRAJECTORY_DURATION;
        }
        
        return dur;
    }

    private void fillPointBuffer()
    {
        if(filled) return;

        // Fills range [startIndex, endIndex)
        int startIndex = 0;
        int endIndex = points.length;
        if(points.length > MAX_POINT_BUFFER_SIZE)
        {
            if(!hasBufferRoom()) return;
            startIndex = fillIndex;
            endIndex = Math.min(startIndex + PERIODIC_BUFFER_FILL_SIZE, points.length);
            fillIndex = endIndex;
        }

        // Cancel previous MP and clear underrun flag if this is the first time filling points
        if(startIndex == 0)
        {
            leftMaster.motor.clearMotionProfileTrajectories();
            leftMaster.motor.clearMotionProfileHasUnderrun(0);
            leftMaster.motor.configMotionProfileTrajectoryPeriod(0, 0); // Set the base trajectory period to 0

            rightMaster.motor.clearMotionProfileTrajectories();
            rightMaster.motor.clearMotionProfileHasUnderrun(0);
            rightMaster.motor.configMotionProfileTrajectoryPeriod(0, 0); // Set the base trajectory period to 0
        }

        TrajectoryPoint point = new TrajectoryPoint();
        for(int i = startIndex; i < endIndex; i++)
        {
            point.position = points[i][0][0];
            point.velocity = points[i][0][1];
            point.timeDur = getTrajectoryDuration((int)points[i][0][2]);
            point.profileSlotSelect0 = pidSlot;
            point.profileSlotSelect1 = pidSlot;
            point.zeroPos = (i == 0);
            point.isLastPoint = (i == points.length-1);

            leftMaster.motor.pushMotionProfileTrajectory(point);

            point.position = points[i][1][0];
            point.velocity = points[i][1][1];
            point.timeDur = getTrajectoryDuration((int)points[i][1][2]);
            point.profileSlotSelect0 = pidSlot;
            point.profileSlotSelect1 = pidSlot;
            point.zeroPos = (i == 0);
            point.isLastPoint = (i == points.length-1);

            rightMaster.motor.pushMotionProfileTrajectory(point);
        }
        if(endIndex >= points.length)
        {
            filled = true;
        }
    }
}

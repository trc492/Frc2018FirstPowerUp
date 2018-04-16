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

    private enum State
    {
        START, WAIT_FOR_POINTS, MONITOR_PATH, DONE
    }

    private String instanceName;
    private PidCoefficients pidCoefficients;
    private int pidSlot;
    private double inchesPerEncoderTick;
    private FrcCANTalon[] leftMotors, rightMotors, allMotors;
    private TrcTaskMgr.TaskObject taskObject;
    private double[][][] points;
    private TrcStateMachine<State> sm;
    private int fillIndex = 0;
    private boolean filled = false;
    private Notifier notifier;
    private MotionProfileStatus[] statuses;

    public FrcSketchyMotionProfile(String instanceName, PidCoefficients pidCoefficients, int pidSlot, double inchesPerEncoderTick)
    {
        this.instanceName = instanceName;
        this.pidCoefficients = pidCoefficients;
        this.pidSlot = pidSlot;
        this.inchesPerEncoderTick = inchesPerEncoderTick;

        sm = new TrcStateMachine<>(instanceName);
        notifier = new Notifier(this::processPointBuffer);

        taskObject = TrcTaskMgr.getInstance().createTask(instanceName + ".postContinuousTask", this::postContinuousTask);
    }

    public void setLeftMotors(FrcCANTalon...leftMotors)
    {
        for(FrcCANTalon talon:leftMotors)
        {
            talon.motor.config_kP(pidSlot, pidCoefficients.kP, 10);
            talon.motor.config_kI(pidSlot, pidCoefficients.kI, 10);
            talon.motor.config_kD(pidSlot, pidCoefficients.kD, 10);
            talon.motor.config_kF(pidSlot, pidCoefficients.kF, 10);

            talon.motor.changeMotionControlFramePeriod(5);
        }
        this.leftMotors = leftMotors;
    }

    public void setRightMotors(FrcCANTalon...rightMotors)
    {
        for(FrcCANTalon talon:rightMotors)
        {
            talon.motor.config_kP(pidSlot, pidCoefficients.kP, 10);
            talon.motor.config_kI(pidSlot, pidCoefficients.kI, 10);
            talon.motor.config_kD(pidSlot, pidCoefficients.kD, 10);
            talon.motor.config_kF(pidSlot, pidCoefficients.kF, 10);

            talon.motor.changeMotionControlFramePeriod(5);
        }
        this.rightMotors = rightMotors;
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

        if(leftMotors == null || rightMotors == null)
        {
            throw new IllegalStateException("Left and right motors must be set before calling start()!");
        }

        allMotors = new FrcCANTalon[leftMotors.length + rightMotors.length];
        int index = 0;
        for(FrcCANTalon talon:leftMotors) allMotors[index++] = talon;
        for(FrcCANTalon talon:rightMotors) allMotors[index++] = talon;

        statuses = new MotionProfileStatus[allMotors.length];
        for(int i = 0; i < statuses.length; i++) statuses[i] = new MotionProfileStatus();

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
        for(FrcCANTalon talon:allMotors)
        {
            talon.motor.changeMotionControlFramePeriod((int)updatePeriod);
        }
    }

    public void cancel()
    {
        notifier.stop();
        sm.stop();
        setTaskEnabled(false);
        if(allMotors != null)
        {
            setTalonValue(SetValueMotionProfile.Disable);
        }
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
                filled = false;
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
                fillPointBuffer();
                setTalonValue(SetValueMotionProfile.Enable); // Keep sending the enable signal
                if(isDone())
                {
                    sm.setState(State.DONE);
                }
                break;

            case DONE:
                cancel();
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
        for(FrcCANTalon talon:allMotors)
        {
            talon.motor.set(ControlMode.MotionProfile, value.value);
        }
    }

    private void fillStatuses()
    {
        for(int i = 0; i < statuses.length; i++)
        {
            allMotors[i].motor.getMotionProfileStatus(statuses[i]);
        }
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
        for(FrcCANTalon talon:allMotors)
        {
            talon.motor.processMotionProfileBuffer();
        }
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
            for(FrcCANTalon talon:allMotors)
            {
                talon.motor.clearMotionProfileTrajectories();
                talon.motor.clearMotionProfileHasUnderrun(0);
                talon.motor.configMotionProfileTrajectoryPeriod(0, 0); // Set the base trajectory period to 0
            }
        }

        TrajectoryPoint point = new TrajectoryPoint();
        for(int i = startIndex; i < endIndex; i++)
        {
            for(FrcCANTalon talon:leftMotors)
            {
                point.position = points[i][0][0];
                point.velocity = points[i][0][1];
                point.timeDur = getTrajectoryDuration((int)points[i][0][2]);
                point.headingDeg = 0;
                point.profileSlotSelect0 = pidSlot;
                point.profileSlotSelect1 = pidSlot;
                point.zeroPos = (i == 0);
                point.isLastPoint = (i == points.length-1);
                
                talon.motor.pushMotionProfileTrajectory(point);
            }
            for(FrcCANTalon talon:rightMotors)
            {
                point.position = points[i][1][0];
                point.velocity = points[i][1][1];
                point.timeDur = getTrajectoryDuration((int)points[i][0][2]);
                point.headingDeg = 0;
                point.profileSlotSelect0 = pidSlot;
                point.profileSlotSelect1 = pidSlot;
                point.zeroPos = (i == 0);
                point.isLastPoint = (i == points.length-1);

                talon.motor.pushMotionProfileTrajectory(point);
            }
        }
        if(endIndex >= points.length)
        {
            filled = true;
        }
    }
}

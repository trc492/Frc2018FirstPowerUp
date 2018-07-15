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

import trclib.TrcEvent;
import trclib.TrcTankMotionProfile;
import trclib.TrcTankMotionProfile.TrcMotionProfilePoint;
import trclib.TrcPidController.PidCoefficients;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTankMotionProfileFollower;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

/**
 * This is an implementation of motion profiling. It streams the profiles to the buffer, and then executes
 * it. Also, the profiles are processed 2x as fast as the first point.
 * This was written by using these resources:
 * https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/Java/MotionProfile/src/org/usfirst/frc/team217/robot
 * https://github.com/CrossTheRoadElec/Phoenix-Documentation/blob/master/Talon%20SRX%20Motion%20Profile%20Reference%20Manual.pdf
 */

public class FrcTankMotionProfileFollower extends TrcTankMotionProfileFollower
{
    private static final double MIN_TRAJ_SECONDS = 1.0; // How many seconds of points to buffer before beginning?
    private static final TrajectoryDuration DEFAULT_TRAJECTORY_DURATION = TrajectoryDuration.Trajectory_Duration_10ms;

    private enum State
    {
        START, WAIT_FOR_POINTS, MONITOR_PATH, DONE
    }

    private PidCoefficients pidCoefficients;
    private int pidSlot;
    private double worldUnitsPerEncoderTick;
    private FrcCANTalon leftMaster, rightMaster;
    private TrcTaskMgr.TaskObject motionProfileTaskObject;
    private TrcTankMotionProfile profile;
    private int numPoints;
    private TrcStateMachine<State> sm;
    private int fillIndex = 0;
    private boolean filled = false;
    private Notifier notifier;
    private MotionProfileStatus[] statuses;
    private boolean cancelled = false;
    private TrcEvent onFinishedEvent;
    private double timedOutTime;
    private int requiredTrajectoryPoints;

    /**
     * Create FrcTankMotionProfileFollower object. Uses default pid slot 0.
     *
     * @param instanceName             Name of the instance, duh.
     * @param pidCoefficients          PidCoefficients object storing the PIDF constants.
     * @param worldUnitsPerEncoderTick Number of word units per encoder tick. For example, inches per encoder tick.
     */
    public FrcTankMotionProfileFollower(String instanceName, PidCoefficients pidCoefficients,
        double worldUnitsPerEncoderTick)
    {
        this(instanceName, pidCoefficients, 0, worldUnitsPerEncoderTick);
    }

    /**
     * Create FrcTankMotionProfileFollower object
     *
     * @param instanceName             Name of the instance, duh.
     * @param pidCoefficients          PidCoefficients object storing the PIDF constants.
     * @param pidSlot                  Index of the pid slot to store the pid constants
     * @param worldUnitsPerEncoderTick Number of word units per encoder tick. For example, inches per encoder tick.
     */
    public FrcTankMotionProfileFollower(String instanceName, PidCoefficients pidCoefficients, int pidSlot,
        double worldUnitsPerEncoderTick)
    {
        super(instanceName);

        this.pidCoefficients = pidCoefficients;
        this.pidSlot = pidSlot;
        this.worldUnitsPerEncoderTick = worldUnitsPerEncoderTick;

        sm = new TrcStateMachine<>(instanceName);
        notifier = new Notifier(this::processPointBuffer);

        motionProfileTaskObject = TrcTaskMgr.getInstance()
            .createTask(instanceName + ".motionProfileTask", this::motionProfileTask);
    }

    /**
     * Sets the motors on the left side of the drive train.
     *
     * @param leftMotors List of motors on the left side of the drive train. The first motor in the list will be used
     *                   as the master motor, and all others will be set as slaves.
     */
    public void setLeftMotors(FrcCANTalon... leftMotors)
    {
        if (leftMotors.length == 0)
        {
            throw new IllegalArgumentException("Cannot pass empty array of motors!");
        }

        this.leftMaster = leftMotors[0];

        leftMaster.motor.config_kP(pidSlot, pidCoefficients.kP);
        leftMaster.motor.config_kI(pidSlot, pidCoefficients.kI);
        leftMaster.motor.config_kD(pidSlot, pidCoefficients.kD);
        leftMaster.motor.config_kF(pidSlot, pidCoefficients.kF);

        if (leftMotors.length > 1)
        {
            for (int i = 1; i < leftMotors.length; i++)
            {
                leftMotors[i].motor.set(ControlMode.Follower, leftMaster.motor.getDeviceID());
            }
        }
    }

    /**
     * Sets the motors on the right side of the drive train.
     *
     * @param rightMotors List of motors on the right side of the drive train. The first motor in the list will be used
     *                    as the master motor, and all others will be set as slaves.
     */
    public void setRightMotors(FrcCANTalon... rightMotors)
    {
        if (rightMotors.length == 0)
        {
            throw new IllegalArgumentException("Cannot pass empty array of motors!");
        }

        this.rightMaster = rightMotors[0];

        rightMaster.motor.config_kP(pidSlot, pidCoefficients.kP);
        rightMaster.motor.config_kI(pidSlot, pidCoefficients.kI);
        rightMaster.motor.config_kD(pidSlot, pidCoefficients.kD);
        rightMaster.motor.config_kF(pidSlot, pidCoefficients.kF);

        if (rightMotors.length > 1)
        {
            for (int i = 1; i < rightMotors.length; i++)
            {
                rightMotors[i].motor.set(ControlMode.Follower, rightMaster.motor.getDeviceID());
            }
        }
    }

    /**
     * Start following the supplied motion profile.
     *
     * @param profile TrcTankMotionProfile object representing the path to follow. Remember to match units with worldUnitsPerEncoderTick!
     * @param event   Event to signal when path has been followed
     * @param timeout Maximum number of seconds to spend following the path. 0.0 means no timeout.
     */
    public void start(TrcTankMotionProfile profile, TrcEvent event, double timeout)
    {
        if (leftMaster == null || rightMaster == null)
        {
            throw new IllegalStateException("Left and right motors must be set before calling start()!");
        }

        this.onFinishedEvent = event;
        if (event != null)
        {
            event.clear();
        }

        this.timedOutTime = -1;
        if (timeout > 0.0)
        {
            this.timedOutTime = TrcUtil.getCurrentTime() + timeout;
        }

        this.profile = profile.copy();
        numPoints = this.profile.getNumPoints();
        this.profile.scale(worldUnitsPerEncoderTick);

        sm.start(State.START);

        statuses = new MotionProfileStatus[] { new MotionProfileStatus(), new MotionProfileStatus() };

        double minDuration = this.profile.getMinTimeStep();

        // Number of points to buffer before beginning MP
        requiredTrajectoryPoints = (int) (MIN_TRAJ_SECONDS / minDuration);

        double updatePeriod = minDuration / 2.0; // 2x as fast as trajectory duration
        notifier.startPeriodic(updatePeriod);

        leftMaster.motor.changeMotionControlFramePeriod((int) (updatePeriod * 1000.0)); // convert seconds to ms
        rightMaster.motor.changeMotionControlFramePeriod((int) (updatePeriod * 1000.0)); // convert seconds to ms
        setTaskEnabled(true);
    }

    /**
     * Is path currently being followed?
     *
     * @return True if yes, false otherwise
     */
    public boolean isActive()
    {
        return sm.isEnabled();
    }

    /**
     * Has this task been cancelled?
     *
     * @return True if someone has called the cancel() method while it was running, false otherwise
     */
    public boolean isCancelled()
    {
        return cancelled;
    }

    /**
     * Stop following the path and cancel the event.
     */
    public void cancel()
    {
        cancelled = true;
        if (onFinishedEvent != null)
            onFinishedEvent.cancel();
        stop();
    }

    /**
     * How many trajectory points are in the bottom buffer of the left talon?
     *
     * @return Number of trajectory points in bottom buffer of left talon. -1 if MP hasn't started.
     */
    public int leftBottomBufferCount()
    {
        return statuses != null ? statuses[0].btmBufferCnt : -1;
    }

    /**
     * How many trajectory points are in the bottom buffer of the right talon?
     *
     * @return Number of trajectory points in bottom buffer of right talon. -1 if MP hasn't started.
     */
    public int rightBottomBufferCount()
    {
        return statuses != null ? statuses[1].btmBufferCnt : -1;
    }

    /**
     * How many trajectory points are in the top buffer of the left talon?
     *
     * @return Number of trajectory points in top buffer of left talon. -1 if MP hasn't started.
     */
    public int leftTopBufferCount()
    {
        return statuses != null ? statuses[0].topBufferCnt : -1;
    }

    /**
     * How many trajectory points are in the top buffer of the right talon?
     *
     * @return Number of trajectory points in top buffer of right talon. -1 if MP hasn't started.
     */
    public int rightTopBufferCount()
    {
        return statuses != null ? statuses[1].topBufferCnt : -1;
    }

    /**
     * Position target for left motor.
     *
     * @return Position target, in world units, for left motor. 0 if MP hasn't started.
     */
    public double leftTargetPosition()
    {
        // convert from ticks to worldUnits
        return leftMaster == null ? 0.0 : leftMaster.motor.getActiveTrajectoryPosition() * worldUnitsPerEncoderTick;
    }

    /**
     * Position target for right motor.
     *
     * @return Position target, in world units, for right motor. 0 if MP hasn't started.
     */
    public double rightTargetPosition()
    {
        // convert from ticks to worldUnits
        return rightMaster == null ? 0.0 : rightMaster.motor.getActiveTrajectoryPosition() * worldUnitsPerEncoderTick;
    }

    /**
     * Velocity target for left motor.
     *
     * @return Velocity target, in world units per second, for left motor. 0 if MP hasn't started.
     */
    public double leftTargetVelocity()
    {
        // convert from ticks/100ms -> worldUnits/sec
        return leftMaster == null ?
            0.0 :
            leftMaster.motor.getActiveTrajectoryVelocity() * worldUnitsPerEncoderTick * 10;
    }

    /**
     * Velocity target for right motor.
     *
     * @return Velocity target, in world units per second, for right motor. 0 if MP hasn't started.
     */
    public double rightTargetVelocity()
    {
        // convert from ticks/100ms -> worldUnits/sec
        return rightMaster == null ?
            0.0 :
            rightMaster.motor.getActiveTrajectoryVelocity() * worldUnitsPerEncoderTick * 10;
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
        if (enabled)
        {
            motionProfileTaskObject.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            motionProfileTaskObject.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }

    private void motionProfileTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        if (!sm.isEnabled())
            return;

        if (timedOutTime != -1 && TrcUtil.getCurrentTime() >= timedOutTime)
        {
            sm.setState(State.DONE);
        }

        State state = sm.getState();
        fillStatuses();

        switch (state)
        {
            case START:
                // Fill the top buffer. If numPoints < MAX_POINT_BUFFER_SIZE, fill completely.
                filled = false;
                cancelled = false;
                fillPointBuffer();
                sm.setState(State.WAIT_FOR_POINTS);
                break;

            case WAIT_FOR_POINTS:
                // Wait for the bottom buffer to get enough trajectory profiles
                if (hasEnoughPoints())
                {
                    sm.setState(State.MONITOR_PATH);
                }
                break;

            case MONITOR_PATH:
                fillPointBuffer(); // Keep filling profiles into top buffer. (only useful if numPoints > MAX_POINT_BUFFER_SIZE)
                setTalonValue(SetValueMotionProfile.Enable); // Keep sending the enable signal
                if (isDone())
                {
                    sm.setState(State.DONE);
                }
                break;

            case DONE:
                stop();
                if (onFinishedEvent != null)
                    onFinishedEvent.set(true);
                break;
        }
    }

    private boolean isDone()
    {
        for (MotionProfileStatus status : statuses)
        {
            if (!status.activePointValid || !status.isLast)
                return false;
        }
        return true;
    }

    private void setTalonValue(SetValueMotionProfile value)
    {
        if (leftMaster != null)
            leftMaster.motor.set(ControlMode.MotionProfile, value.value);
        if (rightMaster != null)
            rightMaster.motor.set(ControlMode.MotionProfile, value.value);
    }

    private void fillStatuses()
    {
        if (leftMaster != null)
            leftMaster.motor.getMotionProfileStatus(statuses[0]);
        if (rightMaster != null)
            rightMaster.motor.getMotionProfileStatus(statuses[1]);
    }

    private boolean hasEnoughPoints()
    {
        for (MotionProfileStatus status : statuses)
        {
            if (status.btmBufferCnt < requiredTrajectoryPoints)
                return false;
        }
        return true;
    }

    private void processPointBuffer()
    {
        if (leftMaster != null)
            leftMaster.motor.processMotionProfileBuffer();
        if (rightMaster != null)
            rightMaster.motor.processMotionProfileBuffer();
    }

    /**
     * Get a TrajectoryDuration object with the specified time
     *
     * @param duration Milliseconds for the duration
     * @return TrajectoryDuration object representing the supplied time, if available
     */
    private TrajectoryDuration getTrajectoryDuration(int duration)
    {
        TrajectoryDuration dur = TrajectoryDuration.valueOf(duration);

        if (dur.value != duration)
        {
            DriverStation.reportError("Duration " + duration + "ms not supported!", false);
            dur = DEFAULT_TRAJECTORY_DURATION;
        }
        return dur;
    }

    private void fillPointBuffer()
    {
        if (filled)
            return;

        // Fills range [startIndex, endIndex)
        int startIndex = fillIndex;
        int endIndex = Math.min(numPoints, startIndex + Math.min(statuses[0].topBufferRem, statuses[1].topBufferRem));
        fillIndex = endIndex;

        // Cancel previous MP and clear underrun flag if this is the first time filling profiles
        if (startIndex == 0)
        {
            leftMaster.motor.clearMotionProfileTrajectories();
            leftMaster.motor.clearMotionProfileHasUnderrun(0);
            leftMaster.motor.configMotionProfileTrajectoryPeriod(0, 0); // Set the base trajectory period to 0

            rightMaster.motor.clearMotionProfileTrajectories();
            rightMaster.motor.clearMotionProfileHasUnderrun(0);
            rightMaster.motor.configMotionProfileTrajectoryPeriod(0, 0); // Set the base trajectory period to 0
        }

        TrajectoryPoint point = new TrajectoryPoint();
        for (int i = startIndex; i < endIndex; i++)
        {
            TrcMotionProfilePoint profilePoint = profile.getLeftPoints()[i];
            point.position = profilePoint.encoderPosition;
            point.velocity = profilePoint.velocity;
            point.timeDur = getTrajectoryDuration((int) (profilePoint.timeStep * 1000)); // Convert from sec to ms
            point.profileSlotSelect0 = pidSlot;
            point.profileSlotSelect1 = pidSlot;
            point.zeroPos = (i == 0);
            point.isLastPoint = (i == numPoints - 1);

            leftMaster.motor.pushMotionProfileTrajectory(point);

            profilePoint = profile.getRightPoints()[i];
            point.position = profilePoint.encoderPosition;
            point.velocity = profilePoint.velocity;
            point.timeDur = getTrajectoryDuration((int) (profilePoint.timeStep * 1000)); // Convert from sec to ms
            point.profileSlotSelect0 = pidSlot;
            point.profileSlotSelect1 = pidSlot;
            point.zeroPos = (i == 0);
            point.isLastPoint = (i == numPoints - 1);

            rightMaster.motor.pushMotionProfileTrajectory(point);
        }
        if (endIndex >= numPoints)
        {
            filled = true;
        }
    }
}
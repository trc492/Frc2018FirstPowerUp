/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

package trclib;

import trclib.TrcTaskMgr.TaskType;

/**
 * This class implements a platform independent simple drive base. The SimpleDriveBase class implements a drive train
 * that may consist of 2 to 6 motors. It supports tank drive and arcade drive with motor stalled detection and
 * inverted drive mode. It also supports gyro assisted drive to keep robot driving straight.
 */
public class TrcSimpleDriveBase
{
    private static final String moduleName = "TrcSimpleDriveBase";
    protected static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    public enum MotorType
    {
        LEFT_FRONT(0),
        RIGHT_FRONT(1),
        LEFT_REAR(2),
        RIGHT_REAR(3),
        LEFT_MID(4),
        RIGHT_MID(5);

        public final int value;

        MotorType(int value)
        {
            this.value = value;
        }
    }   //enum MotorType

    /**
     * This interface is provided by the caller to translate the motor power to actual motor power according to
     * the motor curve. This is useful to linearize the motor performance. This is very useful for many reasons.
     * It could allow the drive base to drive straight by translating wheel power to actual torque. It could also
     * allow us to implement our own ramp rate to limit acceleration and deceleration.
     */
    public interface MotorPowerMapper
    {
        /**
         * This method is called to translate the desired motor power to the actual motor power taking into
         * consideration of the motor torque curve with the current motor speed.
         *
         * @param power specifies the desired motor power.
         * @param speed specifies the current motor speed in the unit of encoder counts per second.
         * @return resulting motor power.
         */
        double translateMotorPower(double power, double speed);
    }   //interface MotorPowerMapper

    private static double DEF_SENSITIVITY = 0.5;
    private static double DEF_MAX_OUTPUT = 1.0;

    protected TrcMotorController leftFrontMotor;
    protected TrcMotorController rightFrontMotor;
    protected TrcMotorController leftRearMotor;
    protected TrcMotorController rightRearMotor;
    private TrcMotorController leftMidMotor;
    private TrcMotorController rightMidMotor;
    protected TrcGyro gyro;
    protected MotorPowerMapper motorPowerMapper = null;
    private int numMotors = 0;
    private double sensitivity = DEF_SENSITIVITY;
    private double maxOutput = DEF_MAX_OUTPUT;
    protected double gyroMaxRotationRate = 0.0;
    protected double gyroAssistKp = 1.0;
    protected boolean gyroAssistEnabled = false;

    private double prevLeftFrontPos = 0.0;
    private double prevRightFrontPos = 0.0;
    private double prevLeftRearPos = 0.0;
    private double prevRightRearPos = 0.0;
    private double lfStallStartTime = 0.0;
    private double rfStallStartTime = 0.0;
    private double lrStallStartTime = 0.0;
    private double rrStallStartTime = 0.0;

    private double yPos;
    private double rotPos;
    private double heading;
    private double yScale;
    private double rotScale;
    private double ySpeed;
    private double turnSpeed;

    protected double lfEnc, rfEnc, lrEnc, rrEnc;
    protected double lfSpeed, rfSpeed, lrSpeed, rrSpeed;

    /**
     * This method is called by different constructors to do common initialization.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftMidMotor specifies the left mid motor of a 6-wheel drive base.
     * @param leftRearMotor specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightMidMotor specifies the right mid motor of a 6-wheel drive base.
     * @param rightRearMotor specifies the right rear motor of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     */
    private void commonInit(
        final TrcMotorController leftFrontMotor, final TrcMotorController leftMidMotor,
        final TrcMotorController leftRearMotor, final TrcMotorController rightFrontMotor,
        final TrcMotorController rightMidMotor, final TrcMotorController rightRearMotor,
        final TrcGyro gyro)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.leftFrontMotor = leftFrontMotor;
        if (leftFrontMotor != null) numMotors++;
        this.rightFrontMotor = rightFrontMotor;
        if (rightFrontMotor != null) numMotors++;

        this.leftRearMotor = leftRearMotor;
        if (leftRearMotor != null) numMotors++;
        this.rightRearMotor = rightRearMotor;
        if (rightRearMotor != null) numMotors++;

        this.leftMidMotor = leftMidMotor;
        if (leftMidMotor != null) numMotors++;
        this.rightMidMotor = rightMidMotor;
        if (rightMidMotor != null) numMotors++;

        this.gyro = gyro;

        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        TrcTaskMgr.TaskObject driveBaseTaskObj = taskMgr.createTask(moduleName + ".driveBaseTask", this::driveBaseTask);
        driveBaseTaskObj.registerTask(TrcTaskMgr.TaskType.STOP_TASK);
        driveBaseTaskObj.registerTask(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);

        yScale = 1.0;
        rotScale = 1.0;
        resetPosition(true);
        resetStallTimer();
    }   //commonInit

    /**
     * Constructor: Create an instance of a 6-wheel drive base.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftMidMotor specifies the left mid motor of a 6-wheel drive base.
     * @param leftRearMotor specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightMidMotor specifies the right mid motor of a 6-wheel drive base.
     * @param rightRearMotor specifies the right rear motor of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     */
    public TrcSimpleDriveBase(
        final TrcMotorController leftFrontMotor, final TrcMotorController leftMidMotor,
        final TrcMotorController leftRearMotor, final TrcMotorController rightFrontMotor,
        final TrcMotorController rightMidMotor, final TrcMotorController rightRearMotor,
        final TrcGyro gyro)
    {
        if (leftFrontMotor == null || rightFrontMotor == null ||
            leftRearMotor == null || rightRearMotor == null ||
            leftMidMotor == null || rightMidMotor == null)
        {
            throw new IllegalArgumentException("All 6 motors must not be null.");
        }
        commonInit(leftFrontMotor, leftMidMotor, leftRearMotor, rightFrontMotor, rightMidMotor, rightRearMotor, gyro);
    }   //TrcSimpleDriveBase

    /**
     * Constructor: Create an instance of a 6-wheel drive base.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftMidMotor specifies the left mid motor of a 6-wheel drive base.
     * @param leftRearMotor specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightMidMotor specifies the right mid motor of a 6-wheel drive base.
     * @param rightRearMotor specifies the right rear motor of the drive base.
     */
    public TrcSimpleDriveBase(
        final TrcMotorController leftFrontMotor, final TrcMotorController leftMidMotor,
        final TrcMotorController leftRearMotor, final TrcMotorController rightFrontMotor,
        final TrcMotorController rightMidMotor, final TrcMotorController rightRearMotor)
    {
        this(leftFrontMotor, leftMidMotor, leftRearMotor, rightFrontMotor, rightMidMotor, rightRearMotor, null);
    }   //TrcSimpleDriveBase

    /**
     * Constructor: Create an instance of a 4-wheel drive base.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftRearMotor specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightRearMotor specifies the right rear motor of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     */
    public TrcSimpleDriveBase(
        final TrcMotorController leftFrontMotor, final TrcMotorController leftRearMotor,
        final TrcMotorController rightFrontMotor, final TrcMotorController rightRearMotor,
        final TrcGyro gyro)
    {
        if (leftFrontMotor == null || rightFrontMotor == null || leftRearMotor == null || rightRearMotor == null)
        {
            throw new IllegalArgumentException("All 4 motors must not be null.");
        }
        commonInit(leftFrontMotor, null, leftRearMotor, rightFrontMotor, null, rightRearMotor, gyro);
    }   //TrcSimpleDriveBase

    /**
     * Constructor: Create an instance of a 4-wheel drive base.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftRearMotor specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightRearMotor specifies the right rear motor of the drive base.
     */
    public TrcSimpleDriveBase(
        final TrcMotorController leftFrontMotor, final TrcMotorController leftRearMotor,
        final TrcMotorController rightFrontMotor, final TrcMotorController rightRearMotor)
    {
        this(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, null);
    }   //TrcSimpleDriveBase

    /**
     * Constructor: Create an instance of a 2-wheel drive base.
     *
     * @param leftMotor specifies the left rear motor of the drive base.
     * @param rightMotor specifies the right rear motor of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     */
    public TrcSimpleDriveBase(
        final TrcMotorController leftMotor, final TrcMotorController rightMotor, final TrcGyro gyro)
    {
        if (leftMotor == null || rightMotor == null)
        {
            throw new IllegalArgumentException("All 2 motors must not be null.");
        }
        commonInit(null, null, leftMotor, null, null, rightMotor, gyro);
    }   //TrcSimpleDriveBase

    /**
     * Constructor: Create an instance of a 2-wheel drive base.
     *
     * @param leftMotor specifies the left rear motor of the drive base.
     * @param rightMotor specifies the right rear motor of the drive base.
     */
    public TrcSimpleDriveBase(final TrcMotorController leftMotor, final TrcMotorController rightMotor)
    {
        this(leftMotor, rightMotor, null);
    }   //TrcSimpleDriveBase

    /**
     * This method sets a motor power mapper. If null, it unsets the previously set mapper.
     *
     * @param motorPowerMapper specifies the motor power mapper. If null, clears the mapper.
     */
    public void setMotorPowerMapper(MotorPowerMapper motorPowerMapper)
    {
        this.motorPowerMapper = motorPowerMapper;
    }   //setMotorPowerMapper

    /**
     * This method sets the sensitivity for the drive() method.
     *
     * @param sensitivity specifies the sensitivity value.
     */
    public void setSensitivity(double sensitivity)
    {
        final String funcName = "setSensitivity";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "sensitivity=%f", sensitivity);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.sensitivity = sensitivity;
    }   //setSensitivity

    /**
     * This method sets the maximum output value of the motor.
     *
     * @param maxOutput specifies the maximum output value.
     */
    public void setMaxOutput(double maxOutput)
    {
        final String funcName = "setMaxOutput";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "maxOutput=%f", maxOutput);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.maxOutput = maxOutput;
    }   //setMaxOutput

    /**
     * This method enables gyro assist drive.
     *
     * @param gyroMaxRotationRate specifies the maximum rotation rate of the robot base reported by the gyro.
     * @param gyroAssistKp specifies the gyro assist proportional constant.
     */
    public void enableGyroAssist(double gyroMaxRotationRate, double gyroAssistKp)
    {
        final String funcName = "enableGyroAssist";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                funcName, TrcDbgTrace.TraceLevel.API, "gyroMaxRate=%f,gyroAssistKp=%f",
                gyroMaxRotationRate, gyroAssistKp);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.gyroMaxRotationRate = gyroMaxRotationRate;
        this.gyroAssistKp = gyroAssistKp;
        this.gyroAssistEnabled = true;
    }   //enableGyroAssist

    /**
     * This method enables/disables gyro assist drive.
     */
    public void disableGyroAssist()
    {
        final String funcName = "enableGyroAssist";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.gyroMaxRotationRate = 0.0;
        this.gyroAssistKp = 1.0;
        this.gyroAssistEnabled = false;
    }   //disableGyroAssist

    /**
     * This method returns the number of motors in the drive train.
     *
     * @return number of motors.
     */
    public int getNumMotors()
    {
        final String funcName = "getNumMotors";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", numMotors);
        }

        return numMotors;
    }   //getNumMotors

    /**
     * This method inverts direction of a given motor in the drive train.
     *
     * @param motorType specifies the motor in the drive train.
     * @param isInverted specifies true if inverting motor direction.
     */
    public void setInvertedMotor(MotorType motorType, boolean isInverted)
    {
        final String funcName = "setInvertedMotor";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "type=%s,inverted=%s", motorType.toString(), Boolean.toString(isInverted));
        }

        switch (motorType)
        {
            case LEFT_FRONT:
                if (leftFrontMotor != null)
                {
                    leftFrontMotor.setInverted(isInverted);
                }
                break;

            case RIGHT_FRONT:
                if (rightFrontMotor != null)
                {
                    rightFrontMotor.setInverted(isInverted);
                }
                break;

            case LEFT_REAR:
                if (leftRearMotor != null)
                {
                    leftRearMotor.setInverted(isInverted);
                }
                break;

            case RIGHT_REAR:
                if (rightRearMotor != null)
                {
                    rightRearMotor.setInverted(isInverted);
                }
                break;

            case LEFT_MID:
                if (leftMidMotor != null)
                {
                    leftMidMotor.setInverted(isInverted);
                }
                break;

            case RIGHT_MID:
                if (rightMidMotor != null)
                {
                    rightMidMotor.setInverted(isInverted);
                }
                break;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setInvertedMotor

    /**
     * This method resets the drive base position odometry. This includes the motor encoders, the gyro heading and
     * all the cached values.
     *
     * @param hardware specifies true for resetting hardware position, false for resetting software position.
     */
    public void resetPosition(boolean hardware)
    {
        final String funcName = "resetPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (leftFrontMotor != null) leftFrontMotor.resetPosition(hardware);
        if (rightFrontMotor != null) rightFrontMotor.resetPosition(hardware);

        if (leftRearMotor != null) leftRearMotor.resetPosition(hardware);
        if (rightRearMotor != null) rightRearMotor.resetPosition(hardware);

        if (leftMidMotor != null) leftMidMotor.resetPosition(hardware);
        if (rightMidMotor != null) rightMidMotor.resetPosition(hardware);

        if (gyro != null) gyro.resetZIntegrator();

        yPos = 0.0;
        rotPos = 0.0;
        heading = 0.0;
        ySpeed = 0.0;
        turnSpeed = 0.0;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetPosition

    /**
     * This method resets the drive base position odometry. This includes the motor encoders, the gyro heading and
     * all the cached values.
     */
    public void resetPosition()
    {
        resetPosition(false);
    }   //resetPosition

    /**
     * This method sets the Y position scale. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param scale specifies the Y position scale.
     */
    public void setYPositionScale(double scale)
    {
        final String funcName = "setYPositionScale";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "scale=%f", scale);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.yScale = scale;
    }   //setYPositionScale

    /**
     * This method sets the rotation scale. This class supports getting the drive base heading even without the gyro
     * by using the difference of the left and right encoders. Again, this would be in encoder counts. By setting
     * the rotation scale, one could get a good approximation of the heading in degrees using encoders only.
     *
     * @param scale specifies the rotation scale.
     */
    public void setRotationScale(double scale)
    {
        final String funcName = "setRotationScale";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "scale=%f", scale);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.rotScale = scale;
    }   //setRotationScale

    /**
     * This method returns the Y position in scaled unit.
     *
     * @return Y position.
     */
    public double getYPosition()
    {
        final String funcName = "getYPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", yPos);
        }

        return yPos;
    }   //getYPosition

    /**
     * This method returns the rotation position in scaled unit.
     *
     * @return rotation position.
     */
    public double getRotatePosition()
    {
        final String funcName = "getRotatePosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", rotPos);
        }

        return rotPos;
    }   //getRotatePosition

    /**
     * This method returns the gyro heading of the drive base in degrees.
     *
     * @return gyro heading.
     */
    public double getHeading()
    {
        final String funcName = "getHeading";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", heading);
        }

        return heading;
    }   //getHeading

    /**
     * This method returns the drive base speed in the Y direction.
     *
     * @return Y speed.
     */
    public double getYSpeed()
    {
        final String funcName = "getYSpeed";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", ySpeed);
        }

        return ySpeed;
    }   //getYSpeed

    /**
     * This method returns the drive base turn speed.
     *
     * @return turn speed.
     */
    public double getTurnSpeed()
    {
        final String funcName = "getTurnSpeed";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", turnSpeed);
        }

        return turnSpeed;
    }   //getTurnSpeed

    public boolean isStalled(MotorType motorType, double stallTime)
    {
        final String funcName = "isStalled";
        double currTime = TrcUtil.getCurrentTime();
        boolean stalled = false;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "motorType=%s,stallTime=%.3f", motorType, stallTime);
        }

        switch (motorType)
        {
            case LEFT_FRONT:
                stalled = currTime - lfStallStartTime > stallTime;
                break;

            case RIGHT_FRONT:
                stalled = currTime - rfStallStartTime > stallTime;
                break;

            case LEFT_MID:
            case LEFT_REAR:
                stalled = currTime - lrStallStartTime > stallTime;
                break;

            case RIGHT_MID:
            case RIGHT_REAR:
                stalled = currTime - rrStallStartTime > stallTime;
                break;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(stalled));
        }

        return stalled;
    }   //isStalled

    /**
     * This method resets the stall timer.
     */
    public void resetStallTimer()
    {
        final String funcName = "resetStallTimer";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        lfStallStartTime = rfStallStartTime = lrStallStartTime = rrStallStartTime = TrcUtil.getCurrentTime();
    }   //resetStallTimer

    /**
     * This method checks if all motors on the drive base have been stalled for at least the specified stallTime.
     *
     * @param stallTime specifies the stall time.
     * @return true if the drive base is stalled, false otherwise.
     */
    public boolean isStalled(double stallTime)
    {
        final String funcName = "isStalled";
        boolean stalled = isStalled(MotorType.LEFT_FRONT, stallTime) && isStalled(MotorType.RIGHT_FRONT, stallTime) &&
                          isStalled(MotorType.LEFT_REAR, stallTime) && isStalled(MotorType.RIGHT_REAR, stallTime);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "stallTime=%.3f", stallTime);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(stalled));
        }

        return stalled;
    }   //isStalled

    /**
     * This method enables/disables brake mode of the drive base.
     *
     * @param enabled specifies true to enable brake mode, false to disable it.
     */
    public void setBrakeMode(boolean enabled)
    {
        final String funcName = "setBrakeMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (leftFrontMotor != null) leftFrontMotor.setBrakeModeEnabled(enabled);
        if (rightFrontMotor != null) rightFrontMotor.setBrakeModeEnabled(enabled);

        if (leftRearMotor != null) leftRearMotor.setBrakeModeEnabled(enabled);
        if (rightRearMotor != null) rightRearMotor.setBrakeModeEnabled(enabled);

        if (leftMidMotor != null) leftMidMotor.setBrakeModeEnabled(enabled);
        if (rightMidMotor != null) rightMidMotor.setBrakeModeEnabled(enabled);
    }   //setBrakeMode

    /**
     * This methods stops the drive base.
     */
    public void stop()
    {
        final String funcName = "stop";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (leftFrontMotor != null) leftFrontMotor.set(0.0);
        if (rightFrontMotor != null) rightFrontMotor.set(0.0);

        if (leftRearMotor != null) leftRearMotor.set(0.0);
        if (rightRearMotor != null) rightRearMotor.set(0.0);

        if (leftMidMotor != null) leftMidMotor.set(0.0);
        if (rightMidMotor != null) rightMidMotor.set(0.0);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //stop

    /**
     * This method drives the motors at "magnitude" and "curve". Both magnitude and curve are -1.0 to +1.0 values,
     * where 0.0 represents stopped and not turning. curve less than 0 will turn left and curve greater than 0 will
     * turn right. The algorithm for steering provides a constant turn radius for any normal speed range, both
     * forward and backward. Increasing sensitivity causes sharper turns for fixed values of curve.
     *
     * @param magnitude specifies the speed setting for the outside wheel in a turn, forward or backwards, +1 to -1.
     * @param curve specifies the rate of turn, constant for different forward speeds. Set curve less than 0 for left
     *              turn or curve greater than 0 for right turn. Set curve = e^(-r/w) to get a turn radius r for
     *              wheel base w of your robot. Conversely, turn radius r = -ln(curve)*w for a given value of curve
     *              and wheel base w.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void drive(double magnitude, double curve, boolean inverted)
    {
        final String funcName = "drive";
        double leftOutput;
        double rightOutput;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "mag=%f,curve=%f,inverted=%s",
                                magnitude, curve, Boolean.toString(inverted));
        }

        if (curve < 0.0)
        {
            double value = Math.log(-curve);
            double ratio = (value - sensitivity)/(value + sensitivity);
            if (ratio == 0.0)
            {
                ratio = 0.0000000001;
            }
            leftOutput = magnitude/ratio;
            rightOutput = magnitude;
        }
        else if (curve > 0.0)
        {
            double value = Math.log(curve);
            double ratio = (value - sensitivity)/(value + sensitivity);
            if (ratio == 0.0)
            {
                ratio = 0.0000000001;
            }
            leftOutput = magnitude;
            rightOutput = magnitude/ratio;
        }
        else
        {
            leftOutput = magnitude;
            rightOutput = magnitude;
        }

        tankDrive(leftOutput, rightOutput, inverted);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //drive

    /**
     * This method drives the motors with the given magnitude and curve values.
     *
     * @param magnitude specifies the magnitude value.
     * @param curve specifies the curve value.
     */
    public void drive(double magnitude, double curve)
    {
        drive(magnitude, curve, false);
    }   //drive

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param leftPower specifies left power value.
     * @param rightPower specifies right power value.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void tankDrive(double leftPower, double rightPower, boolean inverted)
    {
        final String funcName = "tankDrive";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "leftPower=%f,rightPower=%f,inverted=%s",
                                leftPower, rightPower, Boolean.toString(inverted));
        }

        leftPower = TrcUtil.clipRange(leftPower);
        rightPower = TrcUtil.clipRange(rightPower);

        if (inverted)
        {
            double swap = leftPower;
            leftPower = -rightPower;
            rightPower = -swap;
        }

        if (gyroAssistEnabled)
        {
            double diffPower = (leftPower - rightPower)/2.0;
            double assistPower =
                TrcUtil.clipRange(gyroAssistKp*(diffPower - gyro.getZRotationRate().value/gyroMaxRotationRate));
            leftPower += assistPower;
            rightPower -= assistPower;
            double maxMag = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxMag > 1.0)
            {
                leftPower /= maxMag;
                rightPower /= maxMag;
            }
        }

        leftPower = TrcUtil.clipRange(leftPower, -maxOutput, maxOutput);
        rightPower = TrcUtil.clipRange(rightPower, -maxOutput, maxOutput);

        double wheelPower;

        if (leftFrontMotor != null)
        {
            wheelPower = leftPower;
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, leftFrontMotor.getSpeed());
            }
            leftFrontMotor.set(wheelPower);
        }

        if (rightFrontMotor != null)
        {
            wheelPower = rightPower;
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, rightFrontMotor.getSpeed());
            }
            rightFrontMotor.set(wheelPower);
        }

        if (leftRearMotor != null)
        {
            wheelPower = leftPower;
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, leftRearMotor.getSpeed());
            }
            leftRearMotor.set(wheelPower);
        }

        if (rightRearMotor != null)
        {
            wheelPower = rightPower;
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, rightRearMotor.getSpeed());
            }
            rightRearMotor.set(wheelPower);
        }

        if (leftMidMotor != null)
        {
            wheelPower = leftPower;
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, leftMidMotor.getSpeed());
            }
            leftMidMotor.set(wheelPower);
        }

        if (rightMidMotor != null)
        {
            wheelPower = rightPower;
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, rightMidMotor.getSpeed());
            }
            rightMidMotor.set(wheelPower);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //tankDrive

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param leftPower specifies left power value.
     * @param rightPower specifies right power value.
     */
    public void tankDrive(double leftPower, double rightPower)
    {
        tankDrive(leftPower, rightPower, false);
    }   //tankDrive

    /**
     * This method implements arcade drive where drivePower controls how fast the robot goes in the y-axis and
     * turnPower controls how fast it will turn.
     *
     * @param drivePower specifies the drive power value.
     * @param turnPower specifies the turn power value.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void arcadeDrive(double drivePower, double turnPower, boolean inverted)
    {
        final String funcName = "arcadeDrive";
        double leftPower;
        double rightPower;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "drivePower=%f,turnPower=%f,inverted=%s",
                                drivePower, turnPower, Boolean.toString(inverted));
        }

        drivePower = TrcUtil.clipRange(drivePower);
        turnPower = TrcUtil.clipRange(turnPower);

        leftPower = drivePower + turnPower;
        rightPower = drivePower - turnPower;
        double maxMag = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxMag > 1.0)
        {
            leftPower /= maxMag;
            rightPower /= maxMag;
        }

        tankDrive(leftPower, rightPower, inverted);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //arcadeDrive

    /**
     * This method implements arcade drive where drivePower controls how fast the robot goes in the y-axis and
     * turnPower controls how fast it will turn.
     *
     * @param drivePower specifies the drive power value.
     * @param turnPower specifies the turn power value.
     */
    public void arcadeDrive(double drivePower, double turnPower)
    {
        arcadeDrive(drivePower, turnPower, false);
    }   //arcadeDrive

    /**
     * This method normalizes the power to the four wheels for mecanum drive.
     *
     * @param wheelPowers specifies the wheel power of all four wheels.
     */
    protected void normalize(double[] wheelPowers)
    {
        double maxMagnitude = Math.abs(wheelPowers[0]);
        for (int i = 1; i < wheelPowers.length; i++)
        {
            double magnitude = Math.abs(wheelPowers[i]);
            if (magnitude > maxMagnitude)
            {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 1.0)
        {
            for (int i = 0; i < wheelPowers.length; i++)
            {
                wheelPowers[i] /= maxMagnitude;
            }
        }
    }   //normalize

    /**
     * This method is called periodically to monitor the encoders and gyro to update the odometry data or when
     * the competition mode is about to end.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
     */
    public void driveBaseTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "driveBaseTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        if (taskType == TaskType.PRECONTINUOUS_TASK)
        {
            if (leftFrontMotor != null)
            {
                try
                {
                    lfEnc = leftFrontMotor.getPosition();
                    lfSpeed = leftFrontMotor.getSpeed();
                }
                catch (UnsupportedOperationException e)
                {
                    lfEnc = 0.0;
                    lfSpeed = 0.0;
                }
            }

            if (rightFrontMotor != null)
            {
                try
                {
                    rfEnc = rightFrontMotor.getPosition();
                    rfSpeed = rightFrontMotor.getSpeed();
                }
                catch (UnsupportedOperationException e)
                {
                    rfEnc = 0.0;
                    rfSpeed = 0.0;
                }
            }

            if (leftRearMotor != null)
            {
                try
                {
                    lrEnc = leftRearMotor.getPosition();
                    lrSpeed = leftRearMotor.getSpeed();
                }
                catch (UnsupportedOperationException e)
                {
                    lrEnc = 0.0;
                    lrSpeed = 0.0;
                }
            }

            if (rightRearMotor != null)
            {
                try
                {
                    rrEnc = rightRearMotor.getPosition();
                    rrSpeed = rightRearMotor.getSpeed();
                }
                catch (UnsupportedOperationException e)
                {
                    rrEnc = 0.0;
                    rrSpeed = 0.0;
                }
            }

            yPos = (lfEnc + lrEnc + rfEnc + rrEnc)*yScale/numMotors;
            ySpeed = (lfSpeed + lrSpeed + rfSpeed + rrSpeed)*yScale/numMotors;
            rotPos = ((lfEnc + lrEnc) - (rfEnc + rrEnc))*rotScale/numMotors;

            if (gyro != null)
            {
                heading = gyro.getZHeading().value;
                turnSpeed = gyro.getZRotationRate().value;
            }
            else
            {
                heading = rotPos;
            }

            double currTime = TrcUtil.getCurrentTime();
            double lfPower = leftFrontMotor != null? leftFrontMotor.getPower(): 0.0;
            double rfPower = rightFrontMotor != null? rightFrontMotor.getPower(): 0.0;
            double lrPower = leftRearMotor != null? leftRearMotor.getPower(): 0.0;
            double rrPower = rightRearMotor != null? rightRearMotor.getPower(): 0.0;

            if (lfEnc != prevLeftFrontPos || lfPower == 0.0) lfStallStartTime = currTime;
            if (rfEnc != prevRightFrontPos || rfPower == 0.0) rfStallStartTime = currTime;
            if (lrEnc != prevLeftRearPos || lrPower == 0.0) lrStallStartTime = currTime;
            if (rrEnc != prevRightRearPos || rrPower == 0.0) rrStallStartTime = currTime;

            prevLeftFrontPos = lfEnc;
            prevRightFrontPos = rfEnc;
            prevLeftRearPos = lrEnc;
            prevRightRearPos = rrEnc;
        }
        else if (taskType == TaskType.STOP_TASK && runMode != TrcRobot.RunMode.DISABLED_MODE)
        {
            stop();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //driveBaseTask

}   //class TrcSimpleDriveBase
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

package trclib;

/**
 * This class implements a platform independent swerve drive base. A swerve drive base consists of 4 swerve modules
 * each of which consists of a driving motor and a PID controlled steering motor. It extends the TrcSimpleDriveBase
 * class so it inherits all the SimpleDriveBase methods and features
 *
 * The implementation of this code is based on Ether's white paper:
 *  http://www.chiefdelphi.com/media/papers/download/3028
 */
public class TrcSwerveDriveBase extends TrcSimpleDriveBase
{
    private TrcSwerveModule lfModule, rfModule, lrModule, rrModule;
    private double wheelBaseWidth, wheelBaseLength, wheelBaseDiagonal;
    private double positionScale = 1.0;
    private double xEncPos, yEncPos, rotPos;
    private double xEncSpeed, yEncSpeed;
    private Double prevTimestamp = null;
    private double prevLfEnc, prevRfEnc, prevLrEnc, prevRrEnc;

    /**
     * Constructor: Create an instance of the 4-wheel swerve drive base.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftRearMotor specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightRearMotor specifies the right rear motor of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     * @param wheelBaseWidth specifies the width of the wheel base in inches.
     * @param wheelBaseLength specifies the length of the wheel base in inches.
     */
    public TrcSwerveDriveBase(
        TrcSwerveModule leftFrontMotor, TrcSwerveModule leftRearMotor,
        TrcSwerveModule rightFrontMotor, TrcSwerveModule rightRearMotor,
        TrcGyro gyro, double wheelBaseWidth, double wheelBaseLength)
    {
        super(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, gyro);

        this.lfModule = leftFrontMotor;
        this.rfModule = rightFrontMotor;
        this.lrModule = leftRearMotor;
        this.rrModule = rightRearMotor;
        this.wheelBaseWidth = wheelBaseWidth;
        this.wheelBaseLength = wheelBaseLength;
        this.wheelBaseDiagonal = TrcUtil.magnitude(wheelBaseWidth, wheelBaseLength);
        // TODO: do zero calibration on all four wheels.
    }   //TrcSwerveDriveBase

    /**
     * Constructor: Create an instance of the 4-wheel swerve drive base.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftRearMotor specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightRearMotor specifies the right rear motor of the drive base.
     * @param wheelBaseWidth specifies the width of the wheel base in inches.
     * @param wheelBaseLength specifies the length of the wheel base in inches.
     */
    public TrcSwerveDriveBase(
        TrcSwerveModule leftFrontMotor, TrcSwerveModule leftRearMotor,
        TrcSwerveModule rightFrontMotor, TrcSwerveModule rightRearMotor,
        double wheelBaseWidth, double wheelBaseLength)
    {
        this(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, null, wheelBaseWidth, wheelBaseLength);
    }   //TrcSwerveDriveBase

    /**
     * This method checks if it supports holonomic drive.
     *
     * @return true if this drive base supports holonomic drive, false otherwise.
     */
    @Override
    public boolean supportsHolonomicDrive()
    {
        return true;
    }   //supportsHolonomicDrive

    /**
     * This method sets the position scale. In Swerve Drive, there is no separate X and Y position scales.
     * There is just one single position scale that translates the wheels rotation into wheels travel distance.
     * By using sin/cos of the steering angle, one can calculate the traveled distances of X and Y.
     *
     * @param scale specifies the position scale.
     */
    public void setPositionScale(double scale)
    {
        final String funcName = "setPositionScale";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "scale=%f", scale);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        positionScale = scale;
    }   //setPositionScale

    /**
     * This method sets the X position scale.
     *
     * @param scale specifies the X position scale.
     */
    @Override
    public void setXPositionScale(double scale)
    {
        throw new UnsupportedOperationException(
            "Swerve Drive does not have X and Y position scales, please use setPositionScale instead.");
    }   //setXPositionScale

    /**
     * This method sets the Y position scale.
     *
     * @param scale specifies the Y position scale.
     */
    @Override
    public void setYPositionScale(double scale)
    {
        throw new UnsupportedOperationException(
            "Swerve Drive does not have X and Y position scales, please use setPositionScale instead.");
    }   //setYPositionScale

    /**
     * This method sets the rotation scale.
     *
     * @param scale specifies the rotation scale.
     */
    @Override
    public void setRotationScale(double scale)
    {
        throw new UnsupportedOperationException("Swerve Drive does not have Rotation scale.");
    }   //setRotationScale

    /**
     * This method returns the X position in scaled unit.
     *
     * @return X position.
     */
    @Override
    public double getXPosition()
    {
        final String funcName = "getXPosition";
        double pos = xEncPos*positionScale;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", pos);
        }

        return pos;
    }   //getXPosition

    /**
     * This method returns the Y position in scaled unit.
     *
     * @return Y position.
     */
    @Override
    public double getYPosition()
    {
        final String funcName = "getYPosition";
        double pos = yEncPos*positionScale;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", pos);
        }

        return pos;
    }   //getYPosition

    /**
     * This method returns the rotation position in scaled unit.
     *
     * @return rotation position.
     */
    @Override
    public double getRotationPosition()
    {
        final String funcName = "getRotationPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", rotPos);
        }

        return rotPos;
    }   //getRotationPosition

    /**
     * This method returns the drive base speed in the X direction.
     *
     * @return X speed.
     */
    @Override
    public double getXSpeed()
    {
        final String funcName = "getXSpeed";
        double speed = xEncSpeed*positionScale;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", speed);
        }

        return speed;
    }   //getXSpeed

    /**
     * This method returns the drive base speed in the Y direction.
     *
     * @return Y speed.
     */
    @Override
    public double getYSpeed()
    {
        final String funcName = "getYSpeed";
        double speed = yEncSpeed*positionScale;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", speed);
        }

        return speed;
    }   //getYSpeed

    /**
     * This method resets the drive base odometry. This includes the motor encoders, drive base position, speed and
     * gyro heading.
     *
     * @param hardware specifies true for resetting hardware position, false for resetting software position.
     * @param resetGyro specifies true to also reset the gyro heading, false otherwise.
     */
    @Override
    public void resetOdometry(boolean hardware, boolean resetGyro)
    {
        final String funcName = "resetOdometry";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        super.resetOdometry(hardware, resetGyro);
        xEncPos = yEncPos = 0.0;
        xEncSpeed = yEncSpeed = 0.0;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetOdometry

    /**
     * This method sets the steering angle of all four wheels.
     *
     * @param angle specifies the steering angle to be set.
     */
    public void setSteerAngle(double angle)
    {
        final String funcName = "setSteerAngle";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "angle=%f", angle);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        lfModule.setSteerAngle(angle);
        rfModule.setSteerAngle(angle);
        lrModule.setSteerAngle(angle);
        rrModule.setSteerAngle(angle);
    }   //setSteerAngle

    /**
     * This method stops the drive base. If steerNeutral is true, it also sets all steering angles to zero.
     *
     * @param resetSteer specifies true to set steering angle to zero.
     */
    public void stop(boolean resetSteer)
    {
        final String funcName = "stop";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "steerNeutral=%s", resetSteer);
        }

        super.stop();

        if (resetSteer)
        {
            setSteerAngle(0.0);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //stop

    /**
     * This method stops the drive base and reset the steering angle to zero.
     */
    @Override
    public void stop()
    {
        stop(false);
    }   //stop

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors. It will set the steering angle to zero, but note that it will take time for the steering angles to
     * reach zero. Since we do not wait for the steering angle to reach neutral, it is possible the drive base will
     * move diagonally initially. If this is undesirable, the caller should make sure steering angles are already at
     * zero before calling this method.
     *
     * @param leftPower specifies left power value.
     * @param rightPower specifies right power value.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    @Override
    public void tankDrive(double leftPower, double rightPower, boolean inverted)
    {
        setSteerAngle(0.0);
        super.tankDrive(leftPower, rightPower, inverted);
    }   //tankDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the gyro angle to maintain for field relative drive. DO NOT use this with inverted.
     */
    @Override
    public void holonomicDrive(double x, double y, double rotation, boolean inverted, double gyroAngle)
    {
        final String funcName = "holonomicDrive";

        // TODO: need to understand this code.
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "x=%f,y=%f,rot=%f,inverted=%s,angle=%f",
                                x, y, rotation, Boolean.toString(inverted), gyroAngle);
        }

        x = TrcUtil.clipRange(x);
        y = TrcUtil.clipRange(y);
        rotation = TrcUtil.clipRange(rotation);

        if(inverted)
        {
            x = -x;
            y = -y;
        }

        if(gyroAngle != 0)
        {
            if(inverted)
            {
                dbgTrace.traceWarn(
                    funcName, "You should not be using inverted and field reference frame at the same time!");
            }

            double gyroRadians = Math.toRadians(gyroAngle);
            double temp = y * Math.cos(gyroRadians) + x * Math.sin(gyroRadians);
            x = -y * Math.sin(gyroRadians) + x * Math.cos(gyroRadians);
            y = temp;
        }

        double a = x - (rotation * wheelBaseLength/wheelBaseDiagonal);
        double b = x + (rotation * wheelBaseLength/wheelBaseDiagonal);
        double c = y - (rotation * wheelBaseWidth/wheelBaseDiagonal);
        double d = y + (rotation * wheelBaseWidth/wheelBaseDiagonal);

        // The whitepaper goes in order rf, lf, lr, rr. We like to do lf, rf, lr, rr.
        double lfAngle = Math.toDegrees(Math.atan2(b,d));
        double rfAngle = Math.toDegrees(Math.atan2(b,c));
        double lrAngle = Math.toDegrees(Math.atan2(a,d));
        double rrAngle = Math.toDegrees(Math.atan2(a,c));

        // The whitepaper goes in order rf, lf, lr, rr. We like to do lf, rf, lr, rr.
        double lfPower = TrcUtil.magnitude(b, d);
        double rfPower = TrcUtil.magnitude(b, c);
        double lrPower = TrcUtil.magnitude(a, d);
        double rrPower = TrcUtil.magnitude(a, c);

        double[] normalizedPowers = TrcUtil.normalize(lfPower, rfPower, lrPower, rrPower);
        lfPower = this.clipMotorOutput(normalizedPowers[0]);
        rfPower = this.clipMotorOutput(normalizedPowers[1]);
        lrPower = this.clipMotorOutput(normalizedPowers[2]);
        rrPower = this.clipMotorOutput(normalizedPowers[3]);

        lfModule.setSteerAngle(lfAngle);
        rfModule.setSteerAngle(rfAngle);
        lrModule.setSteerAngle(lrAngle);
        rrModule.setSteerAngle(rrAngle);

        lfModule.set(motorPowerMapper.translateMotorPower(lfPower, lfModule.getSpeed()));
        rfModule.set(motorPowerMapper.translateMotorPower(rfPower, rfModule.getSpeed()));
        lrModule.set(motorPowerMapper.translateMotorPower(lrPower, lrModule.getSpeed()));
        rrModule.set(motorPowerMapper.translateMotorPower(rrPower, rrModule.getSpeed()));

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //holonomicDrive

    /**
     * This method is called periodically to monitor the encoders to update the odometry data.
     */
    @Override
    protected void updateOdometry()
    {
        final String funcName = "updateOdometry";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK);
        }

        double currTime = TrcUtil.getCurrentTime();

        double lfEnc = lfModule.getPosition();
        double rfEnc = rfModule.getPosition();
        double lrEnc = lrModule.getPosition();
        double rrEnc = rrModule.getPosition();

        double lfAngle = lfModule.getSteerAngle();
        double rfAngle = rfModule.getSteerAngle();
        double lrAngle = lrModule.getSteerAngle();
        double rrAngle = rrModule.getSteerAngle();

        double avgAngleDeg = TrcUtil.average(lfAngle, rfAngle, lrAngle, rrAngle);
        double avgAngleRad = Math.toRadians(avgAngleDeg);
        double angleCos = Math.cos(avgAngleRad);
        double angleSin = Math.sin(avgAngleRad);

        //
        // First time. Initialize all prevEnc readings.
        //
        // Make timestamp nullable instead.
        if (prevTimestamp == null)
        {
            prevTimestamp = currTime;
            prevLfEnc = lfEnc;
            prevRfEnc = rfEnc;
            prevLrEnc = lrEnc;
            prevRrEnc = rrEnc;
        }

        double timeDelta = currTime - prevTimestamp;
        double avgEncDelta = TrcUtil.average(
            lfEnc - prevLfEnc, rfEnc - prevRfEnc, lrEnc - prevLrEnc, rrEnc - prevRrEnc);

        xEncPos += avgEncDelta*angleCos;
        yEncPos += avgEncDelta*angleSin;
        rotPos = avgAngleDeg;   //TODO: Think about this some more???

        double avgEncSpeed = timeDelta != 0.0? avgEncDelta/timeDelta: 0.0;
        xEncSpeed = avgEncSpeed*angleCos;
        yEncSpeed = avgEncSpeed*angleSin;

        prevTimestamp = currTime;
        prevLfEnc = lfEnc;
        prevRfEnc = rfEnc;
        prevLrEnc = lrEnc;
        prevRrEnc = rrEnc;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //updateOdometry

}   //class TrcSwerveDriveBase

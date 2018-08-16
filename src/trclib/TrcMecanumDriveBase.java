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

import trclib.TrcTaskMgr.TaskType;

/**
 * This class implements a platform independent mecanum drive base. A mecanum drive base consists of 4 motor driven
 * wheels. It extends the TrcSimpleDriveBase class so it inherits all the SimpleDriveBase methods and features.
 */
public class TrcMecanumDriveBase extends TrcSimpleDriveBase
{
    private double xPos = 0.0;
    private double xScale = 1.0;
    private double xSpeed = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftRearMotor specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightRearMotor specifies the right rear motor of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     */
    public TrcMecanumDriveBase(
        TrcMotor leftFrontMotor, TrcMotor leftRearMotor, TrcMotor rightFrontMotor, TrcMotor rightRearMotor,
        TrcGyro gyro)
    {
        super(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, gyro);
    }   //TrcMecanumDriveBase

    /**
     * Constructor: Create an instance of the object.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftRearMotor specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightRearMotor specifies the right rear motor of the drive base.
     */
    public TrcMecanumDriveBase(
        TrcMotor leftFrontMotor, TrcMotor leftRearMotor, TrcMotor rightFrontMotor, TrcMotor rightRearMotor)
    {
        super(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, null);
    }   //TrcMecanumDriveBase

    /**
     * This method resets the drive base position odometry. This includes the motor encoders, the gyro heading and
     * all the cached values.
     *
     * @param hardware specifies true for resetting hardware position, false for resetting software position.
     */
    @Override
    public void resetPosition(boolean hardware)
    {
        super.resetPosition(hardware);

        xPos = 0.0;
        xSpeed = 0.0;
    }   //resetPosition

    /**
     * This method sets the X position scale. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param scale specifies the X position scale.
     */
    public void setXPositionScale(double scale)
    {
        final String funcName = "setXPositionScale";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "scale=%f", scale);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.xScale = scale;
    }   //setXPositionScale

    /**
     * This method returns the X position in scaled unit.
     *
     * @return X position.
     */
    public double getXPosition()
    {
        final String funcName = "getXPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", xPos);
        }

        return xPos;
    }   //getXPosition

    /**
     * This method returns the drive base speed in the X direction.
     *
     * @return X speed.
     */
    public double getXSpeed()
    {
        final String funcName = "getXSpeed";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", xSpeed);
        }

        return xSpeed;
    }   //getXSpeed

    /**
     * This method implements mecanum drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the gyro angle to maintain.
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation, boolean inverted, double gyroAngle)
    {
        final String funcName = "mecanumDrive_Cartesian";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "x=%f,y=%f,rot=%f,inverted=%s,angle=%f",
                                x, y, rotation, Boolean.toString(inverted), gyroAngle);
        }

        x = TrcUtil.clipRange(x);
        y = TrcUtil.clipRange(y);
        rotation = TrcUtil.clipRange(rotation);

        if (inverted)
        {
            x = -x;
            y = -y;
        }

        double cosA = Math.cos(Math.toRadians(gyroAngle));
        double sinA = Math.sin(Math.toRadians(gyroAngle));
        double x1 = x*cosA - y*sinA;
        double y1 = x*sinA + y*cosA;

        if (gyroAssistEnabled)
        {
            double zRotationRate = gyro.getZRotationRate().value;
            double normalizedRotationRate = zRotationRate/gyroMaxRotationRate;
            double error = rotation - normalizedRotationRate;
            rotation += TrcUtil.clipRange(gyroAssistKp*error);
            if(debugEnabled)
            {
                dbgTrace.traceInfo("mecanumDrive_Cartesian", 
                    "Gyro assist: rotationTarget=%.3f normalizedRotationRate=%.3f", rotation, normalizedRotationRate);
            }
        }

        double wheelPowers[] = new double[4];
        wheelPowers[MotorType.LEFT_FRONT.value] = x1 + y1 + rotation;
        wheelPowers[MotorType.RIGHT_FRONT.value] = -x1 + y1 - rotation;
        wheelPowers[MotorType.LEFT_REAR.value] = -x1 + y1 + rotation;
        wheelPowers[MotorType.RIGHT_REAR.value] = x1 + y1 - rotation;
        normalize(wheelPowers);

        double wheelPower;

        if (leftFrontMotor != null)
        {
            wheelPower = wheelPowers[MotorType.LEFT_FRONT.value];
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, leftFrontMotor.getSpeed());
            }
            leftFrontMotor.set(wheelPower);
        }

        if (rightFrontMotor != null)
        {
            wheelPower = wheelPowers[MotorType.RIGHT_FRONT.value];
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, rightFrontMotor.getSpeed());
            }
            rightFrontMotor.set(wheelPower);
        }

        if (leftRearMotor != null)
        {
            wheelPower = wheelPowers[MotorType.LEFT_REAR.value];
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, leftRearMotor.getSpeed());
            }
            leftRearMotor.set(wheelPower);
        }

        if (rightRearMotor != null)
        {
            wheelPower = wheelPowers[MotorType.RIGHT_REAR.value];
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, rightRearMotor.getSpeed());
            }
            rightRearMotor.set(wheelPower);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //mecanumDrive_Cartesian

    /**
     * This method implements mecanum drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation, boolean inverted)
    {
        mecanumDrive_Cartesian(x, y, rotation, inverted, 0.0);
    }   //mecanumDrive_Cartesian

    /**
     * This method implements mecanum drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates.
     *
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation)
    {
        mecanumDrive_Cartesian(x, y, rotation, false, 0.0);
    }   //mecanumDrive_Cartesian

    /**
     * This method implements mecanum drive where magnitude controls how fast the robot will go in the given direction
     * and how fast it will rotate.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation specifies the rotation power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void mecanumDrive_Polar(double magnitude, double direction, double rotation, boolean inverted)
    {
        final String funcName = "mecanumDrive_Polar";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "mag=%f,dir=%f,rot=%f,inverted=%s",
                                magnitude, direction, rotation, Boolean.toString(inverted));
        }

        magnitude = TrcUtil.clipRange(magnitude) * Math.sqrt(2.0);
        if (inverted)
        {
            direction += 180.0;
            direction %= 360.0;
        }

        double dirInRad = Math.toRadians(direction + 45.0);
        double cosD = Math.cos(dirInRad);
        double sinD = Math.sin(dirInRad);

        if (gyroAssistEnabled)
        {
            rotation += TrcUtil.clipRange(gyroAssistKp*(rotation - gyro.getZRotationRate().value/gyroMaxRotationRate));
        }

        double wheelPowers[] = new double[4];
        wheelPowers[MotorType.LEFT_FRONT.value] = (sinD*magnitude + rotation);
        wheelPowers[MotorType.RIGHT_FRONT.value] = (cosD*magnitude - rotation);
        wheelPowers[MotorType.LEFT_REAR.value] = (cosD*magnitude + rotation);
        wheelPowers[MotorType.RIGHT_REAR.value] = (sinD*magnitude - rotation);
        normalize(wheelPowers);

        double wheelPower;

        if (leftFrontMotor != null)
        {
            wheelPower = wheelPowers[MotorType.LEFT_FRONT.value];
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, leftFrontMotor.getSpeed());
            }
            leftFrontMotor.set(wheelPower);
        }

        if (rightFrontMotor != null)
        {
            wheelPower = wheelPowers[MotorType.RIGHT_FRONT.value];
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, rightFrontMotor.getSpeed());
            }
            rightFrontMotor.set(wheelPower);
        }

        if (leftRearMotor != null)
        {
            wheelPower = wheelPowers[MotorType.LEFT_REAR.value];
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, leftRearMotor.getSpeed());
            }
            leftRearMotor.set(wheelPower);
        }

        if (rightRearMotor != null)
        {
            wheelPower = wheelPowers[MotorType.RIGHT_REAR.value];
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, rightRearMotor.getSpeed());
            }
            rightRearMotor.set(wheelPower);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //mecanumDrive_Polar

    /**
     * This method implements mecanum drive where magnitude controls how fast the robot will go in the given direction
     * and how fast it will rotate.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation specifies the rotation power.
     */
    public void mecanumDrive_Polar(double magnitude, double direction, double rotation)
    {
        mecanumDrive_Polar(magnitude, direction, rotation, false);
    }   //mecanumDrive_Polar

    /**
     * This method is called periodically to monitor the encoders and gyro to update the odometry data or when
     * the competition mode is about to end.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
     */
    @Override
    public void driveBaseTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        super.driveBaseTask(taskType, runMode);

        if (taskType == TaskType.PRECONTINUOUS_TASK)
        {
            //
            // According to RobotDrive.mecanumDrive_Cartesian in WPILib:
            //
            // LF =  x + y + rot    RF = -x + y - rot
            // LR = -x + y + rot    RR =  x + y - rot
            //
            // (LF + RR) - (RF + LR) = (2x + 2y) - (-2x + 2y)
            // => (LF + RR) - (RF + LR) = 4x
            // => x = ((LF + RR) - (RF + LR))/4
            //
            // LF + RF + LR + RR = 4y
            // => y = (LF + RF + LR + RR)/4
            //
            // (LF + LR) - (RF + RR) = (2y + 2rot) - (2y - 2rot)
            // => (LF + LR) - (RF + RR) = 4rot
            // => rot = ((LF + LR) - (RF + RR))/4
            //
            xPos = ((lfEnc + rrEnc) - (rfEnc + lrEnc))*xScale/4.0;
            xSpeed = ((lfSpeed + rrSpeed) - (rfSpeed + lrSpeed))*xScale/4.0;
        }
    }   //driveBaseTask

}   //class TrcMecanumDriveBase

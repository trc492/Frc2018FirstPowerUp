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

package frclib;

import com.ctre.phoenix.motorcontrol.ControlMode;

import trclib.TrcEvent;
import trclib.TrcPidController.PidCoefficients;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcTimer;

/**
 * This is a super sketchy implementation of motion magic that probably won't work. This was made by using
 * examples and info from phoenix docs and examples.
 */

public class FrcSketchyMotionMagic
{
    private FrcCANTalon[] leftMotors, rightMotors;
    private double[][] path;
    private int posIndex = 0;
    private TrcTaskMgr.TaskObject pathTaskObj;
    private double timestep;
    private TrcTimer timer;
    private TrcEvent timerEvent;
    
    private int pidSlot, velocity, acceleration;
    private PidCoefficients pidCoefficients;
    private double inchesPerEncoderTick;
    public FrcSketchyMotionMagic(String instanceName, PidCoefficients pidCoefficients,
                                 int pidSlot, int velocity, int acceleration, double inchesPerEncoderTick)
    {
        this.pidSlot = pidSlot;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.pidCoefficients = pidCoefficients;
        this.inchesPerEncoderTick = inchesPerEncoderTick;

        pathTaskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".pathTask", this::followPathTask);
        timer = new TrcTimer(instanceName);
        timerEvent = new TrcEvent(instanceName);
    }
    
    public void setLeftMotors(FrcCANTalon...talons)
    {
        this.leftMotors = talons;
        for(FrcCANTalon talon:talons)
        {
            talon.resetPosition(true);
            
            talon.motor.configMotionCruiseVelocity(velocity,10);
            talon.motor.configMotionAcceleration(acceleration,10);
            
            talon.motor.config_kP(pidSlot, pidCoefficients.kP, 10);
            talon.motor.config_kI(pidSlot, pidCoefficients.kI, 10);
            talon.motor.config_kD(pidSlot, pidCoefficients.kD, 10);
            talon.motor.config_kF(pidSlot, pidCoefficients.kF, 10);
        }
    }
    
    public void setRightMotors(FrcCANTalon...talons)
    {
        this.rightMotors = talons;
        for(FrcCANTalon talon:talons)
        {
            talon.resetPosition(true);
            
            talon.motor.configMotionCruiseVelocity(velocity,10);
            talon.motor.configMotionAcceleration(acceleration,10);
            
            talon.motor.config_kP(pidSlot, pidCoefficients.kP, 10);
            talon.motor.config_kI(pidSlot, pidCoefficients.kI, 10);
            talon.motor.config_kD(pidSlot, pidCoefficients.kD, 10);
            talon.motor.config_kF(pidSlot, pidCoefficients.kF, 10);
        }
    }

    /**
     * Start following the supplied path. Only supports tank drive.
     * @param path 2d array of shape [numPoints, 2]. Uses inches.
     * @param timestep Number of seconds to wait between each point. (Ex: 0.05)
     */
    public void start(final double[][] path, double timestep)
    {
        if(path[0].length != 2)
        {
            throw new IllegalArgumentException("path must be 2d array with size [numPoints, 2]!");
        }
        
        if(leftMotors == null || rightMotors == null)
        {
            throw new IllegalStateException("Left and right motors must be set before calling start()!");
        }
        
        this.path = path;
        this.timestep = timestep;
        posIndex = 0;

        // Convert from inches to encoder position
        for(double[] point:path)
        {
            point[0] /= inchesPerEncoderTick;
            point[1] /= inchesPerEncoderTick;
        }

        setPathTaskEnabled(true);
    }

    public void cancel()
    {
        setPathTaskEnabled(false);

        for(FrcCANTalon talon:leftMotors)
        {
            talon.setPower(0.0);
        }

        for(FrcCANTalon talon:rightMotors)
        {
            talon.setPower(0.0);
        }
    }

    private void setPathTaskEnabled(boolean enabled)
    {
        if(enabled)
        {
            pathTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            timer.set(timestep, timerEvent);
        }
        else
        {
            pathTaskObj.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            timer.cancel();
        }
    }

    private void followPathTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        for(FrcCANTalon talon:leftMotors)
        {
            talon.motor.set(ControlMode.MotionMagic, path[posIndex][0]);
        }

        for(FrcCANTalon talon:rightMotors)
        {
            talon.motor.set(ControlMode.MotionMagic, path[posIndex][1]);
        }

        if(timerEvent.isSignaled())
        {
            posIndex++;

            // If reached the last point, stop
            if(posIndex == path.length)
            {
                setPathTaskEnabled(false);
            }
            else
            {
                timer.set(timestep, timerEvent);
            }
        }
    }
}
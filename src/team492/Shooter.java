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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frclib.FrcCANTalon;

public class Shooter
{
    private FrcCANTalon motor;
    double setPoint = 0.0;

    public Shooter()
    {
        motor = new FrcCANTalon("Shooter", RobotInfo.CANID_SHOOTER);
        motor.setInverted(false);
        motor.motor.overrideLimitSwitchesEnable(false);
        motor.setPositionSensorInverted(true);
        motor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
//        motor.motor.configEncoderCodesPerRev(RobotInfo.SHOOTER_COUNT_PER_REV);
    }

    public void setPID(double p, double i, double d, double f, int izone, double closeLoopRampRate, int profile)
    {
        motor.motor.config_kP(0, p, 0);
        motor.motor.config_kI(0, i, 0);
        motor.motor.config_kD(0, d, 0);
        motor.motor.config_kF(0, f, 0);
//        motor.motor.setPID(p, i, d, f, izone, closeLoopRampRate, profile);
    }

    public double getSetPoint()
    {
        return setPoint;
    }

    public double getSpeed()
    {
        return motor.motor.getSelectedSensorVelocity(0);
    }

    public void setSpeed(double rpm)
    {
        //
        // counts_per_second = rpm*counts_per_rev/60
        // counts_per_10msec = rpm*counts_per_rev*100/60
        //
        setPoint = rpm*100.0/60.0;
        motor.motor.set(ControlMode.Position, setPoint);
    }

    public double getPower()
    {
        return motor.getPower();
    }

    public void setPower(double power)
    {
        motor.motor.set(ControlMode.Velocity, power);
    }

    public double getCurrent()
    {
        return motor.motor.getOutputCurrent();
    }

}   //class Shooter

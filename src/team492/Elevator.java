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

package team492;

import trclib.TrcEvent;
import trclib.TrcMotor;
import trclib.TrcPidActuator;
import trclib.TrcPidController;
import frclib.FrcCANTalon;
import trclib.TrcDigitalInput;

public class Elevator
{
    public TrcPidActuator elevator;
    public TrcMotor elevatorMotor;
    public TrcPidController elevatorPidCtrl;
    public TrcDigitalInput elevatorLowerLimitSwitch;
//S    private TrcAnalogInput dogLeash = null; // this needs to be decided later.

    private double elevatorPower = 0.0;

    public Elevator()
    {
        //CodeReview: where do you initialize elevatorLowerLimitSwitch? There may also be an upperLimitSwitch.
        // We are currently using soft limit switch, no need.
        elevatorMotor = new FrcCANTalon("elevatorMotor", RobotInfo.ELEVATOR_MOTOR_ID); // the name and the device number
        elevatorPidCtrl = new TrcPidController(
            "elevatorPidController",
            new TrcPidController.PidCoefficients(RobotInfo.ELEVATOR_KP, RobotInfo.ELEVATOR_KI, RobotInfo.ELEVATOR_KD),
            RobotInfo.ELEVATOR_TOLERANCE, elevator::getPosition);
        elevator = new TrcPidActuator(
            "elevator", elevatorMotor, elevatorLowerLimitSwitch, elevatorPidCtrl,
            RobotInfo.ELEVATOR_MIN_HEIGHT, RobotInfo.ELEVATOR_MAX_HEIGHT);
    }

    public void zeroCalibrate()
    {
        elevator.zeroCalibrate(RobotInfo.ELEVATOR_CAL_POWER);
    }   //zeroCalibrate

    /**
     * 
     * @param pos (Altitude in inches)
     * @param event (TrcEvent event)
     * @param timeout 
     * Set the position for Elevator to move to in inches using PID control.
     */
    public void setPosition(double pos, TrcEvent event, double timeout)
    {
        elevator.setTarget(pos, event, timeout);
    }   //setPosition

    public void setPower(double power)
    {
        double pos = getPosition();

        if (power > 0.0 && pos >= RobotInfo.ELEVATOR_MAX_HEIGHT ||
            power < 0.0 && pos <= RobotInfo.ELEVATOR_MIN_HEIGHT)
        {
            power = 0.0;
        }

        elevator.setPower(power);
        elevatorPower = power;
    }   //setPower

    // get the current power the elevator actuator is running at.
    public double getPower()
    {
        return elevatorPower;
    }

    // get the current altitude of the elevator relative to encoder zero. (in inches)
    public double getPosition()
    {
        return elevator.getPosition();
    }
    
}

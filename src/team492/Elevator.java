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
	
    static final double ELEVATOR_INCHES_PER_COUNT      = 0.002822426329889;  // we will need to change this, later during calibration
    static final double ELEVATOR_KP                      = 0.3;                // this too
    static final double ELEVATOR_KI                      = 0.0;                // hopefully not this
    static final double ELEVATOR_KD                      = 0.0;                // this too
    static final double ELEVATOR_TOLERANCE              = 0.5;				  // this too
    static final double ELEVATOR_MIN_HEIGHT             = 0.0;             
    static final double ELEVATOR_MAX_HEIGHT             = 85.0;              
    static final double ELEVATOR_MID_HEIGHT             = 4.0;
    static final double ELEVATOR_CAL_POWER              = 0.3;               // this too
    static final double ELEVATOR_SENSOR_ZERO_OFFSET    = -0.529;            // this too
    static final double ELEVATOR_SENSOR_SCALE           = 11.18586208869386; // and this
	
    
	public TrcPidActuator elevator;
	public TrcMotor elevatorMotor;
	public TrcPidController elevatorPidCtrl;
	public TrcDigitalInput elevatorLowerLimitSwitch;
	
	public Elevator()
	{
		elevatorMotor = new FrcCANTalon("elevatorMotor", RobotInfo.ELEVATOR_MOTOR_ID); // the name and the device number
		elevator = new TrcPidActuator(
                "elevator", elevatorMotor, elevatorLowerLimitSwitch, elevatorPidCtrl,
                ELEVATOR_MIN_HEIGHT, ELEVATOR_MAX_HEIGHT);

	}
	
	public void zeroCalibrate()
    {
        elevator.zeroCalibrate(ELEVATOR_CAL_POWER);
    }   //zeroCalibrate

    public void setPosition(double pos, TrcEvent event, double timeout)
    {
        elevator.setTarget(pos, event, timeout);
    }   //setPosition
}

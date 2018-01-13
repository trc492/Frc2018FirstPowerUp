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

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import trclib.TrcRobotBattery;

/**
 * This class extends the TrcRobotBattery which provides a task to monitor the robot battery levels and the methods to
 * access the highest and the lowest battery levels during the monitoring session.
 */
public class FrcRobotBattery extends TrcRobotBattery
{
    private PowerDistributionPanel pdp;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param module specifies the CAN ID of the PDP.
     */
    public FrcRobotBattery(int module)
    {
        super();
        pdp = new PowerDistributionPanel(module);
    }   //FrcRobotBattery

    /**
     * Constructor: Creates an instance of the object.
     */
    public FrcRobotBattery()
    {
        this(0);
    }   //FrcRobotBattery

    //
    // Implements TrcRobotBattery abstract methods.
    //

    /**
     * This method returns the robot battery voltage.
     *
     * @return robot battery voltage in volts.
     */
    @Override
    public double getVoltage()
    {
        return pdp.getVoltage();
    }   //getVoltage

    /**
     * This method returns the robot battery current.
     *
     * @return robot battery current in amps.
     */
    @Override
    public double getCurrent()
    {
        return pdp.getTotalCurrent();
    }   //getCurrent

    /**
     * This method returns the robot battery power.
     *
     * @return robot battery current in watts.
     */
    @Override
    public double getPower()
    {
        return pdp.getTotalPower();
    }   //getPower

}   //class FrcRobotBattery

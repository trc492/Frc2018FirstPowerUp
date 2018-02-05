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

import frclib.FrcDigitalInput;
import trclib.TrcDigitalTrigger;
import trclib.TrcEvent;
import trclib.TrcRobot;

public class CmdPortalAlign implements TrcRobot.RobotCommand
{
	private static final String moduleName = "CmdPortalAlign";
	
	private Robot robot;
	private TrcEvent pidEvent, leftEvent, rightEvent;
	private boolean isRight;
	private boolean done;
	private FrcDigitalInput leftProximitySensor, rightProximitySensor;
	private TrcDigitalTrigger leftTrigger, rightTrigger;
	
	public CmdPortalAlign(Robot robot)
	{
		this.robot = robot;
		pidEvent = new TrcEvent(moduleName);
		leftEvent = new TrcEvent(moduleName);
		rightEvent = new TrcEvent(moduleName);
		
		leftProximitySensor = new FrcDigitalInput("LeftProximitySensor", RobotInfo.DIN_LEFT_PROXIMITY_SENSOR);
		rightProximitySensor = new FrcDigitalInput("RightProximitySensor", RobotInfo.DIN_RIGHT_PROXIMITY_SENSOR);
		
		leftTrigger = new TrcDigitalTrigger("LeftSensorTrigger", leftProximitySensor, this::leftTriggerEvent);
		rightTrigger = new TrcDigitalTrigger("RightSensorTrigger", rightProximitySensor, this::rightTriggerEvent);
	}
	
	/**
	 * Set which way the robot will strafe
	 * @param isRight If true, strafe right. If false, strafe left
	 */
	public void setDirection(boolean isRight)
	{
		this.isRight = isRight;
	}
	
	/**
	 * Will the robot strafe right?
	 * @return True if the robot is set to strafe right. False if the robot is set to strafe left.
	 */
	public boolean getDirection()
	{
		return isRight;
	}
	
	/**
	 * Start auto portal alignment. Identical to calling setDirection(isRight) before start()
	 * @param isRight If true, strafe right. If false, strafe left
	 */
	public void start(boolean isRight)
	{
		setDirection(isRight);
		start();
	}
	
	/**
	 * Start auto portal alignment
	 */
	public void start()
	{
		done = false;
		leftTrigger.setEnabled(true);
		rightTrigger.setEnabled(true);
		double relativeDirection = RobotInfo.PORTAL_ALIGN_STRAFE_DIST * (isRight ? 1.0 : -1.0);
		robot.pidDrive.setTarget(relativeDirection, 0.0, robot.targetHeading, false, pidEvent);		
	}
	
	/**
	 * Stop auto portal alignment
	 */
	public void stop()
	{
		done = true;
		leftTrigger.setEnabled(false);
		rightTrigger.setEnabled(false);
		robot.pidDrive.cancel();
	}
	
	@Override
	public boolean cmdPeriodic(double elapsedTime)
	{
		if(done)
		{
			return true;
		}
		
		if(pidEvent.isSignaled() || isRight && leftEvent.isSignaled() || !isRight && rightEvent.isSignaled())
		{
			stop();
		}
		
		return done;
	}

	private void leftTriggerEvent(boolean enabled)
	{
		leftEvent.set(true);
		leftTrigger.setEnabled(false);
	}
	
	private void rightTriggerEvent(boolean enabled)
	{
		rightEvent.set(true);
		leftTrigger.setEnabled(false);
	}
}

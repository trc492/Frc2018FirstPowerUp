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
import trclib.TrcRobot.RunMode;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcTaskMgr.TaskType;

public class CmdAutoCubePickup implements TrcTaskMgr.Task
{
    private static final String moduleName = "CmdAutoCubePickup";
    private static final TaskType taskType = TaskType.POSTCONTINUOUS_TASK;

    private static enum State
    {
        START, DRIVE, PICKUP, DONE
    }

    private TrcEvent event, onFinishedEvent;
    private TrcStateMachine<State> sm;
    private Robot robot;
    private double startTime;
    private double startX, startY;

    public CmdAutoCubePickup(Robot robot)
    {
    	this.robot = robot;
    	
    	event = new TrcEvent(moduleName);
    	sm = new TrcStateMachine<>(moduleName);
    }
    
    /**
     * Enable or disable this Task. When disabled, startTask() and stopTask() will NOT do anything
     * @param enabled
     */
    private void setEnabled(boolean enabled)
    {
    	TrcTaskMgr taskManager = TrcTaskMgr.getInstance();
    	if(enabled)
    	{
    		taskManager.registerTask(moduleName, this, taskType);
    	} else
    	{
    		taskManager.unregisterTask(this, taskType);
    	}
    }

    /**
     * Is the state machine enabled?
     * @return Is the state machine enabled?
     */
    public boolean isEnabled()
    {
        return sm.isEnabled();
    }
    
    public double elapsedTime()
    {
    	return Robot.getModeElapsedTime() - startTime;
    }
    
    public double[] distanceMoved()
    {
    	double changeX = robot.driveBase.getXPosition() - startX;
    	double changeY = robot.driveBase.getYPosition() - startY;
    	return new double[] {changeX, changeY };
    }
	
	public void start()
	{
		start(null);
	}
	
	/**
	 * Start this task, and signal onFinishedEvent when done
	 * @param onFinishedEvent TrcEvent to signal when finished
	 */
	public void start(TrcEvent onFinishedEvent)
	{
		this.onFinishedEvent = onFinishedEvent;
		startTime = Robot.getModeElapsedTime();
		startX = robot.driveBase.getXPosition();
		startY = robot.driveBase.getYPosition();
		setEnabled(true);
		sm.start(State.START);
	}
	
	public void stop()
	{
		if (robot.visionPidDrive.isActive())
        {
            robot.visionPidDrive.cancel();
        }
        sm.stop();
        setEnabled(false);
	}
	
	@Override
	public void startTask(RunMode runMode) {}
	
	@Override
	public void stopTask(RunMode runMode) {}
	
	@Override
	public void prePeriodicTask(RunMode runMode) {}

	@Override
	public void postPeriodicTask(RunMode runMode) {}

	@Override
	public void preContinuousTask(RunMode runMode) {}

	@Override
	public void postContinuousTask(RunMode runMode) {
		boolean done = !sm.isEnabled();
		
		if (done)
		{
			return;
		}
		
		double elapsedTime = Robot.getModeElapsedTime();
		
		State state = sm.getState();
		robot.dashboard.displayPrintf(1, "State: %s", state != null ? state.toString() : "Disabled");
		
		if (sm.isReady())
		{
			switch (state)
			{
				case START:
					// Deploy and open cube pickup
					robot.cubePickup.deployPickup();
					robot.cubePickup.openClaw();
	
					// 6.0 inches above the ground. pos is in inches
					double pos = RobotInfo.ELEVATOR_FLOOR_PICKUP_HEIGHT;
					robot.elevator.setPosition(pos, event, 0.0);
					
					sm.waitForSingleEvent(event, State.DRIVE);
					break;
	
				case DRIVE:
					robot.visionPidDrive.driveMaintainHeading(0.0, 0.6, 0.0); // Go forward at 60% power facing the cube
					robot.cubePickup.grabCube(0.5, event);
	
					sm.waitForSingleEvent(event, State.PICKUP);
					break;
	
				case PICKUP:
					robot.visionPidDrive.cancel();
					robot.cubePickup.closeClaw();
					sm.setState(State.DONE);
					break;
	
				case DONE:
				default:
					stopTask(null);
					done = true;
					if(onFinishedEvent != null)
					{
						onFinishedEvent.set(true);
					}
					break;
			}
			robot.traceStateInfo(elapsedTime, state.toString());
		}
	}
}

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

import team492.PixyVision.TargetInfo;
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

    public CmdAutoCubePickup(Robot robot)
    {
    	this.robot = robot;
    	
    	event = new TrcEvent(moduleName);
    	sm = new TrcStateMachine<>(moduleName);    	
    }
    
    public void setEnabled(boolean enabled)
    {
    	TrcTaskMgr taskManager = TrcTaskMgr.getInstance();
    	if(enabled)
    	{
    		taskManager.registerTask(moduleName, this, taskType);
    	} else
    	{
    		this.stopTask(null);
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

    /**
     * Start this task without signaling any event when done
     */
	@Override
	public void startTask(RunMode runMode) {
		startTaskWithEvent(null);
	}
	
	/**
	 * Start this task, and signal onFinishedEvent when done
	 * @param onFinishedEvent TrcEvent to signal when finished
	 */
	public void startTaskWithEvent(TrcEvent onFinishedEvent)
	{
		this.onFinishedEvent = onFinishedEvent;
		TrcTaskMgr.getInstance().registerTask(moduleName, this, taskType);
	}

	/**
	 * Stop this task but don't deregister it
	 */
	@Override
	public void stopTask(RunMode runMode) {
		if (robot.visionPidDrive.isActive())
        {
            robot.visionPidDrive.cancel();
        }
        sm.stop();
	}
	
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
					TargetInfo target = getCubeCoords();
					robot.visionPidDrive.setTarget(target.xDistance*1.1, target.yDistance*1.1, 0.0, false, event); // Scale by 110% to overshoot
	
					robot.cubePickup.grabCube(0.5, event);
	
					sm.waitForSingleEvent(event, State.PICKUP);
					break;
	
				case PICKUP:
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
					TrcTaskMgr.getInstance().unregisterTask(this, taskType);
					break;
			}
			robot.traceStateInfo(elapsedTime, state.toString());
		}
	}
	
	private TargetInfo getCubeCoords()
	{
		TargetInfo target = robot.pixy.getTargetInfo();
		return target;
	}
}

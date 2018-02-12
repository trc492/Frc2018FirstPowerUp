package team492;

import team492.PixyVision.TargetInfo;
import trclib.TrcEvent;
import trclib.TrcRobot.RunMode;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcTaskMgr.Task;
import trclib.TrcTaskMgr.TaskType;
public class CmdStrafeUntilCube implements Task {
	private static final String moduleName = "CmdStrafeUntilCube";
	private static final TaskType taskType = TaskType.POSTCONTINUOUS_TASK;
	
	private enum State
	{
		START_STRAFE, CHECK_FOR_CUBE, DONE
	}
	
	private Robot robot;
	private TrcStateMachine<State> sm;
	private TrcEvent onFinishedEvent;
	private StopTrigger stopTrigger;
	private boolean strafeRight;
	private double startTime;
	private double startX, startY;
	
	public CmdStrafeUntilCube(Robot robot)
	{
		this.robot = robot;
		sm = new TrcStateMachine<>(moduleName);
	}
	
	/**
	 * Start the task strafing in the specified direction.
	 * @param strafeRight If true, strafe right. If false, strafe left.
	 */
	public void start(boolean strafeRight)
	{
		start(strafeRight, null);
	}
	
	/**
	 * Start the task strafing in the specified direction. Signal event when done.
	 * @param strafeRight If true, strafe right. If false, strafe left.
	 * @param onFinishedEvent When cube is in sight near the center, signal this event
	 */
	public void start(boolean strafeRight, TrcEvent onFinishedEvent)
	{
		start(strafeRight, onFinishedEvent, this::shouldStop);
	}
	
	/**
	 * 
	 * @param strafeRight
	 * @param onFinishedEvent
	 * @param stopTrigger Should return true when this should stop
	 */
	public void start(boolean strafeRight, TrcEvent onFinishedEvent, StopTrigger stopTrigger)
	{
		this.strafeRight = strafeRight;
		this.onFinishedEvent = onFinishedEvent;
		sm.start(State.START_STRAFE);
		setEnabled(true);
		
		startTime = Robot.getModeElapsedTime();
		startX = robot.driveBase.getXPosition();
		startY = robot.driveBase.getYPosition();
	}
	
	/**
	 * Default stopTrigger. Continue indefinitely.
	 */
	private boolean shouldStop(double elapsedTime, double changeX, double changeY)
	{
		return false;
	}
	
	/**
	 * Cancel the task. Use distanceMoved() to see how much the robot has moved
	 */
	public void stop()
	{
		robot.pidDrive.cancel();
		sm.stop();
		setEnabled(false);
	}
	
	/**
	 * Is this task current active?
	 * @return True if active, false if not.
	 */
	public boolean isRunning()
	{
		return sm.isEnabled();
	}
	
	/**
	 * How much time has passed since this task was started?
	 * @return Time in seconds
	 */
    public double elapsedTime()
    {
    	return Robot.getModeElapsedTime() - startTime;
    }
    
    /**
     * Return distance moved since this task was started
     * @return scaled distance
     */
    public double[] distanceMoved()
    {
    	return new double[] { changeX(), changeY() };
    }
    
    private double changeX()
    {
    	return robot.driveBase.getXPosition() - startX;    	
    }
    
    private double changeY()
    {
    	return robot.driveBase.getYPosition() - startY;    	
    }
	
	private void setEnabled(boolean enabled)
	{
		if(enabled)
		{
			TrcTaskMgr.getInstance().registerTask(moduleName, this, taskType);
		} else
		{
			TrcTaskMgr.getInstance().unregisterTask(this, taskType);
		}
	}

	@Override
	public void postContinuousTask(RunMode runMode) {
		State state = sm.getState();
		robot.dashboard.displayPrintf(1, "State: %s", state != null ? state.toString() : "Disabled");
		
		if(isRunning() && stopTrigger.shouldStop(elapsedTime(), changeX(), changeY()))
		{
			stop();
		}
		
		if (sm.isReady())
		{
			switch (state)
			{
				case START_STRAFE:
					double xPower = RobotInfo.FIND_CUBE_STRAFE_POWER * (strafeRight?1:-1);
					robot.pidDrive.driveMaintainHeading(xPower, 0.0, robot.targetHeading);
					sm.setState(State.CHECK_FOR_CUBE);
					break;
				case CHECK_FOR_CUBE:
					TargetInfo target = robot.pixy.getTargetInfo();
					if(target != null && Math.abs(target.angle) <= RobotInfo.FIND_CUBE_MAX_ANGLE)
					{
						sm.setState(State.DONE);
					}
					break;
				case DONE:
					robot.pidDrive.cancel();
					if(onFinishedEvent != null)
					{
						onFinishedEvent.set(true);
					}
					break;
			}
			robot.traceStateInfo(Robot.getModeElapsedTime(), state.toString());
		}
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
	
	public interface StopTrigger{
		public boolean shouldStop(double elapsedTime, double changeX, double changeY);
	}

}

package team492;

import team492.PixyVision.TargetInfo;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

public class CmdAutoCubePickup implements TrcRobot.RobotCommand
{
	private static final String moduleName = "CmdAutoCubePickup";
	private static enum State
	{
		START,
		DRIVE,
		PICKUP,
		DONE
	}
	
	private TrcEvent event;
	private TrcStateMachine<State> sm;
	private Robot robot;
	public CmdAutoCubePickup(Robot robot)
	{
		this.robot = robot;
		
		event = new TrcEvent(moduleName);
		sm = new TrcStateMachine<>(moduleName);
	}
	
	public void start() {
		stop();
		sm.start(State.START);
	}
	
	public void stop() {
		if(robot.visionPidDrive.isActive()) {
			robot.visionPidDrive.cancel();
		}
		sm.stop();
	}
	
	public boolean isEnabled()
	{
		return sm.isEnabled();
	}
	
	@Override
	public boolean cmdPeriodic(double elapsedTime)
	{
		boolean done = !sm.isEnabled();
		if(done) return true;
		State state = sm.getState();
		robot.dashboard.displayPrintf(1, "State: %s", state != null? state.toString(): "Disabled");
		
		if(sm.isReady()) {
			switch (state){
				case START:
					robot.cubePickup.openClaw(); //TODO: Talk to Chris about singular method to init Claw subystem
					double pos = 6.0 * (1.0 / Elevator.ELEVATOR_INCHES_PER_COUNT); // Convert 6.0 inches to encoder ticks
					// Assuming pos parameter of setPosition is in ticks
					robot.elevator.setPosition(pos, event, 5.0); // Need to tune timeout section of this
					sm.waitForSingleEvent(event, State.DRIVE);
					break;
					
				case DRIVE:
					TargetInfo target = getCubeCoords();
					robot.visionPidDrive.setTarget(target.xDistance, target.yDistance, 0d,
							true, event);
					// robot.cubePickup.setCubeTriggerEnabled(true)
					// TODO: Talk to Chris to provide TrcDigitalTrigger to monitor proximity sensor
					sm.waitForSingleEvent(event, State.PICKUP);
					break;
				
				case PICKUP:
					robot.cubePickup.closeClaw(); // TODO: Talk to Chris about singular method to grab cube
					sm.setState(State.DONE);
					break;
				
				case DONE:
				default:
					sm.stop();
					done = true;
					break;
			}
			robot.traceStateInfo(elapsedTime, state.toString());
		}
		return done;
	}
	
	private TargetInfo getCubeCoords()
	{
		TargetInfo target = robot.pixy.getTargetInfo();
		return target;
	}

}

package team492;

import trclib.TrcRobot;
import trclib.TrcStateMachine;

public class CmdPickupCube implements TrcRobot.RobotCommand{
	private static final String moduleName = "CmdPickupCube";
	private static enum State
	{
		GRAB_CUBE,
		WAIT_FOR_CUBE,
		DONE
	}   //enum State
	
	private TrcStateMachine<State> sm;
	private Robot robot;
	
	CmdPickupCube(Robot robot)
    {
        this.robot = robot;

        sm = new TrcStateMachine<>(moduleName);
    }   //CmdPickupCube

	
	public void start() {
		sm.start(State.GRAB_CUBE);
	}
	
	@Override
	public boolean cmdPeriodic(double elapsedTime)
    {
		boolean done = !sm.isEnabled();
		State state = sm.getState();
		if (sm.isReady())
		{
			state = sm.getState();
			switch (state)
			{
			case GRAB_CUBE:
				//
				//  Spins motors to pull in cube.
				//
				robot.cubePickup.grabCube(0.5);
				sm.setState(State.WAIT_FOR_CUBE);
				break;

			case WAIT_FOR_CUBE:
				//
				// Continues spinning motors until a cube is detected.
				//
				if(robot.cubePickup.cubeDetected()) {
					sm.setState(State.DONE);
				}
				else {
					sm.setState(State.WAIT_FOR_CUBE);
				}
				break;

			case DONE:
			default:
				//
				// Done.
				//
				robot.cubePickup.stopPickup();
				sm.stop();
				break;
			}
		}
		return done;
    }

}

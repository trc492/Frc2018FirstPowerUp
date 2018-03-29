package team492.diagnostics.tests;

import team492.Elevator;
import team492.diagnostics.OnBoardDiagnostics.Subsystem;

public class ElevatorPositionUnchangedTest extends ExpectSensorChangeTest {
	private static final int MIN_EXPECTED_MOVE_INCHES = 1;

	public ElevatorPositionUnchangedTest(String name, Elevator elevator) {
		super(name, Subsystem.ELEVATOR, elevator::getPosition, MIN_EXPECTED_MOVE_INCHES); 
	}

	@Override
	public String getErrorMessage() {
		return "elevator did not move enough - encoder might be unplugged";
	}
}

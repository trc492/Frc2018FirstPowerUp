package team492.diagnostics;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import team492.Robot;
import team492.diagnostics.DiagnosticsTest.TestResult;
import team492.diagnostics.tests.DigitalSensorUnchangedTest;
import team492.diagnostics.tests.ElevatorPositionUnchangedTest;
import team492.diagnostics.tests.EncoderUnpluggedTest;
import team492.diagnostics.tests.UltrasonicUnpluggedTest;

public class OnBoardDiagnostics {
	
	private List<DiagnosticsTest> tests;
	
	public OnBoardDiagnostics(Robot robot)
	{
		tests = new ArrayList<>();
		tests.add(new EncoderUnpluggedTest(robot.leftFrontWheel, "left front encoder"));
		tests.add(new EncoderUnpluggedTest(robot.rightFrontWheel, "right front encoder"));
		tests.add(new EncoderUnpluggedTest(robot.leftRearWheel, "left rear encoder"));
		tests.add(new EncoderUnpluggedTest(robot.rightRearWheel, "right rear encoder"));
		
		tests.add(new UltrasonicUnpluggedTest(robot::getLeftSonarDistance, "left sonar"));
		tests.add(new UltrasonicUnpluggedTest(robot::getRightSonarDistance, "right sonar"));
		
		tests.add(new DigitalSensorUnchangedTest(
				robot.elevator.elevatorMotor::isLowerLimitSwitchActive, 
				"elevator lower limit switch"));
		
		tests.add(new DigitalSensorUnchangedTest(
				robot.cubePickup::cubeInProximity, 
				"grabber cube proximity sensor"));

		tests.add(new ElevatorPositionUnchangedTest(robot.elevator));
	}
	
	public void doPeriodicTests()
	{
		tests.forEach(DiagnosticsTest::test);
	}
	
	public void printDiagnostics()
	{
		boolean faults = false;
		
		for(DiagnosticsTest test : tests) {
			TestResult result = test.getResult();
			
			if(result.faultDetected())
			{
				faults = true;
				DriverStation.reportError("### Diagnostics: " + result.errorMessage(), false);
			}
		}
		
		if(!faults)
		{
			DriverStation.reportError("### Diagnostics: No faults", false);
		}
	}
}

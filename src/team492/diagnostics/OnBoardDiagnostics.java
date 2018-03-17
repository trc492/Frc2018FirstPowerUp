package team492.diagnostics;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import team492.Robot;
import team492.diagnostics.DiagnosticsTest.TestResult;

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

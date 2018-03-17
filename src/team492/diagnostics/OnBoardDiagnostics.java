package team492.diagnostics;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import team492.Robot;
import team492.diagnostics.DiagnosticsTest.TestResult;

public class OnBoardDiagnostics {
	
	private List<DiagnosticsTest> tests;
	
	public OnBoardDiagnostics(Robot robot)
	{
		tests = new ArrayList<>();
		tests.add(new EncoderUnplugged(robot.leftFrontWheel, "left front encoder"));
		tests.add(new EncoderUnplugged(robot.rightFrontWheel, "right front encoder"));
		tests.add(new EncoderUnplugged(robot.leftRearWheel, "left rear encoder"));
		tests.add(new EncoderUnplugged(robot.rightRearWheel, "right rear encoder"));
	}
	
	public void printDiagnostics()
	{
		boolean faults = false;
		
		for(DiagnosticsTest test : tests) {
			TestResult result = test.test();
			
			if(result.faultDetected())
			{
				faults = true;
				System.out.println(result.errorMessage());
			}
		}
		
		if(!faults)
		{
			System.out.println("no faults");
		}
	}

}

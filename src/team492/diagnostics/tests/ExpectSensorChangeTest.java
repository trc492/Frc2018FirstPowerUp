package team492.diagnostics.tests;

import java.util.function.Supplier;

import team492.diagnostics.DiagnosticsTest;
import team492.diagnostics.OnBoardDiagnostics.Subsystem;

public abstract class ExpectSensorChangeTest extends DiagnosticsTest {
	
	private final Supplier<Double> sensor;
	private final double expectedMinAbsChange;
	
	private boolean sensorWorking = false;
	private double firstReading;
	
	public ExpectSensorChangeTest(String name, Subsystem subsystem, Supplier<Double> sensor, double expectedMinAbsChange) 
	{
	    super(name, subsystem);
		this.sensor = sensor;
		this.expectedMinAbsChange = expectedMinAbsChange;
		this.firstReading = sensor.get();
	}

	@Override
	public void test() {
		double currentReading = sensor.get();
		if(Math.abs(currentReading - firstReading) >= expectedMinAbsChange)
		{
			sensorWorking = true;
		}
	}

	@Override
	public TestResult getResult() {
		return new TestResult(!sensorWorking, getErrorMessage());
	}
	
	public abstract String getErrorMessage();
}

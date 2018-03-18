package team492.diagnostics.tests;

import java.util.function.Supplier;

import team492.diagnostics.DiagnosticsTest;

public abstract class ExpectSensorChangeTest implements DiagnosticsTest {
	
	private final Supplier<Double> sensor;
	private final double expectedMinAbsChange;
	
	private boolean sensorWorking = false;
	private double firstReading;
	
	public ExpectSensorChangeTest(Supplier<Double> sensor, double expectedMinAbsChange) 
	{
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

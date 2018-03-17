package team492.diagnostics;

import java.util.function.Supplier;

public class UltrasonicUnpluggedTest implements DiagnosticsTest{

	boolean sensorWorking = false;
	double firstSonarReading;
	Supplier<Double> sonarSensor;
	String sensorName;
	
	public UltrasonicUnpluggedTest(Supplier<Double> sonarSensor, String sensorName) 
	{
		this.sonarSensor = sonarSensor;
		firstSonarReading = sonarSensor.get();
		this.sensorName = sensorName;
	}

	@Override
	public void test() {
		double currentSonarReading = sonarSensor.get();
		if(Math.abs(currentSonarReading - firstSonarReading) < 1.0)
		{
			sensorWorking = true;
		}
	}

	@Override
	public TestResult getResult() {
		return new TestResult(!sensorWorking, this.sensorName + " might be unplugged");
	}

}

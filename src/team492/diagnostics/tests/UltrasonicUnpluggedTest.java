package team492.diagnostics.tests;

import java.util.function.Supplier;

public class UltrasonicUnpluggedTest extends ExpectSensorChangeTest {

	private static final double EXPECTED_CHANGE_INCHES = 1.0;
	
	public UltrasonicUnpluggedTest(Supplier<Double> sonarSensor, String sensorName) 
	{
		super(sensorName, sonarSensor, EXPECTED_CHANGE_INCHES);
	}

	@Override
	public String getErrorMessage() {
		return this.getName() + " did not change enough and might be unplugged";
	}
}

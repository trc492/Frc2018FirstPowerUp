package team492.diagnostics.tests;

import java.util.function.Supplier;

public class UltrasonicUnpluggedTest extends ExpectSensorChangeTest {

	private static final double EXPECTED_CHANGE_INCHES = 1.0;
	private final String sensorName;
	
	public UltrasonicUnpluggedTest(Supplier<Double> sonarSensor, String sensorName) 
	{
		super(sonarSensor, EXPECTED_CHANGE_INCHES);
		this.sensorName = sensorName;
	}

	@Override
	public String getErrorMessage() {
		return sensorName + " did not change enough and might be unplugged";
	}
}

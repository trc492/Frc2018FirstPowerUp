package team492.diagnostics.tests;

import java.util.function.Supplier;

import team492.diagnostics.OnBoardDiagnostics.Subsystem;

public class UltrasonicUnpluggedTest extends ExpectSensorChangeTest {

	private static final double EXPECTED_CHANGE_INCHES = 6.0;
	
	public UltrasonicUnpluggedTest(Supplier<Double> sonarSensor, String sensorName) 
	{
		super(sensorName, Subsystem.SENSORS, sonarSensor, EXPECTED_CHANGE_INCHES);
	}

	@Override
	public String getErrorMessage() {
		return this.getName() + " did not change enough and might be unplugged";
	}
}

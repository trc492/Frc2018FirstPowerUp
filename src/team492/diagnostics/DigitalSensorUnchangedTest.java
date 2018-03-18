package team492.diagnostics;

import java.util.function.Supplier;

public class DigitalSensorUnchangedTest extends ExpectSensorChangeTest {
	
	private final String name;

	public DigitalSensorUnchangedTest(Supplier<Boolean> sensor, String name) {
		// Map boolean sensor to double values to reuse ExpectSensorChangeTest
		super(() -> sensor.get() ? 1.0 : 0.0, 1.0);
		this.name = name;
	}

	@Override
	public String getErrorMessage() {
		return name + " never changed states / might be unplugged";
	}
}

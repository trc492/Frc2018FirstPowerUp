package team492.diagnostics.tests;

import java.util.function.Supplier;

public class DigitalSensorUnchangedTest extends ExpectSensorChangeTest {
	public DigitalSensorUnchangedTest(Supplier<Boolean> sensor, String name) {
		// Map boolean sensor to double values to reuse ExpectSensorChangeTest
		super(name, () -> sensor.get() ? 1.0 : 0.0, 1.0);
	}

	@Override
	public String getErrorMessage() {
		return this.getName() + " never changed states / might be unplugged";
	}
}

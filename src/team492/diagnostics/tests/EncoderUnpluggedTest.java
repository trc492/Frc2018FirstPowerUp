package team492.diagnostics.tests;

import frclib.FrcCANTalon;

public class EncoderUnpluggedTest extends ExpectSensorChangeTest{

	private static final double EXPECTED_DRIVEBASE_ENCODER_CHANGE_INCHES = 10.0;
	private String encoderName;
	
	public EncoderUnpluggedTest(FrcCANTalon motor, String encoderName)
	{
		super(motor::getPosition, EXPECTED_DRIVEBASE_ENCODER_CHANGE_INCHES);
		this.encoderName = encoderName;

	}

	@Override
	public String getErrorMessage() {
		return this.encoderName + " did not change enough / might be unplugged";
	}
}

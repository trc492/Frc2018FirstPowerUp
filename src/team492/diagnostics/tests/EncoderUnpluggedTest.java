package team492.diagnostics.tests;

import frclib.FrcCANTalon;
import team492.diagnostics.OnBoardDiagnostics.Subsystem;

public class EncoderUnpluggedTest extends ExpectSensorChangeTest{

	private static final double EXPECTED_DRIVEBASE_ENCODER_CHANGE_INCHES = 10.0;
	
	public EncoderUnpluggedTest(FrcCANTalon motor, String encoderName, Subsystem subsystem)
	{
		super(encoderName, subsystem, motor::getPosition, EXPECTED_DRIVEBASE_ENCODER_CHANGE_INCHES);

	}

	@Override
	public String getErrorMessage() {
		return this.getName() + " did not change enough / might be unplugged";
	}
}

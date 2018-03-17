package team492.diagnostics;

import frclib.FrcCANTalon;

public class EncoderUnplugged implements DiagnosticsTest{

	private FrcCANTalon motor;
	private String encoderName;
	
	public EncoderUnplugged(FrcCANTalon motor, String encoderName)
	{
		this.motor = motor;
		this.encoderName = encoderName;
	}
	
	@Override
	public TestResult test() {
		boolean faulted = motor.getPosition() == 0.0;
		return new TestResult(faulted, this.encoderName + " might be unplugged");
	}
	

}

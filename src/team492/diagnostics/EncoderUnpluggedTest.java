package team492.diagnostics;

import frclib.FrcCANTalon;

public class EncoderUnpluggedTest implements DiagnosticsTest{

	private FrcCANTalon motor;
	private String encoderName;
	
	public EncoderUnpluggedTest(FrcCANTalon motor, String encoderName)
	{
		this.motor = motor;
		this.encoderName = encoderName;
	}
	
	@Override
	public TestResult getResult() {
		boolean faulted = motor.getPosition() == 0.0;
		return new TestResult(faulted, this.encoderName + " might be unplugged");
	}
	
	@Override
	public void test()
	{
		
	}

}

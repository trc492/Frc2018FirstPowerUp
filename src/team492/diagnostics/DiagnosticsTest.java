package team492.diagnostics;

import team492.diagnostics.OnBoardDiagnostics.Subsystem;

public abstract class DiagnosticsTest {
    
    private String name;
    private Subsystem subsystem;
    
    public DiagnosticsTest(String name, Subsystem subsystem)
    {
        this.name = name;
        this.subsystem = subsystem;
    }
    
    public String getName()
    {
        return name;
    }
    
    @Override
    public String toString()
    {
        return name;
    }
    
    public Subsystem getSubsystem()
    {
        return subsystem;
    }
    
	public abstract void test();
	public abstract TestResult getResult();
	
	public static class TestResult {
		private boolean fault;
		private String errorMessage;
		
		public TestResult(boolean isFaulty, String errorMessage) {
			this.fault = isFaulty;
			this.errorMessage = errorMessage;
		}
		
		public boolean faultDetected() {
			return this.fault;
		}
		
		public String errorMessage() {
			return this.errorMessage;
		}
	}
}

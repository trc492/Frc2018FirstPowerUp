package team492.diagnostics;

public abstract class DiagnosticsTest {
    
    private String name;
    public DiagnosticsTest(String name)
    {
        this.name = name;
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

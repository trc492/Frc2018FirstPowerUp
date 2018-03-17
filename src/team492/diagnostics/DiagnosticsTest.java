package team492.diagnostics;

public interface DiagnosticsTest {
	public void test();
	public TestResult getResult();
	
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

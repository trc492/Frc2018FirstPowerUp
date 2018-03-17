package team492.diagnostics;

public interface DiagnosticsTest {
	public TestResult test();
	
	public static class TestResult {
		private boolean fault;
		private String errorMessage;
		
		public TestResult(boolean fault, String errorMessage) {
			this.fault = fault;
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

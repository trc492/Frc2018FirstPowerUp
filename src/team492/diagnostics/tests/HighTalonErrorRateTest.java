package team492.diagnostics.tests;

import frclib.FrcCANTalon;
import team492.diagnostics.DiagnosticsTest;

public class HighTalonErrorRateTest implements DiagnosticsTest {

    private static final int ERROR_COUNT_THRESHOLD = 100;

    private final FrcCANTalon motor;
    private final String motorName;

    public HighTalonErrorRateTest(FrcCANTalon motor, String motorName) {
        this.motor = motor;
        this.motorName = motorName;
    }

    @Override
    public void test() {
    }

    @Override
    public TestResult getResult() {
        final boolean tooManyErrors = motor.getErrorCount() > ERROR_COUNT_THRESHOLD;

        return new TestResult(tooManyErrors, String.format("%s reported more than %d errors - " +
                "there might be a problem with the CAN wiring",
                motorName, ERROR_COUNT_THRESHOLD));
    }
}

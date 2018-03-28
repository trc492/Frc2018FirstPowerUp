package team492.diagnostics.tests;

import frclib.FrcCANTalon;
import team492.diagnostics.DiagnosticsTest;

public class HighTalonErrorRateTest extends DiagnosticsTest {

    private static final int ERROR_COUNT_THRESHOLD = 100;

    private final FrcCANTalon motor;

    public HighTalonErrorRateTest(FrcCANTalon motor, String motorName) {
        super(motorName);
        this.motor = motor;
    }

    @Override
    public void test() {
    }

    @Override
    public TestResult getResult() {
        final boolean tooManyErrors = motor.getErrorCount() > ERROR_COUNT_THRESHOLD;

        return new TestResult(tooManyErrors, String.format("%s reported more than %d errors - " +
                "there might be a problem with the CAN wiring",
                this.getName(), ERROR_COUNT_THRESHOLD));
    }
}

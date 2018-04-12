package team492.diagnostics;

import frclib.FrcCANTalon;
import team492.OnBoardDiagnostics.Subsystem;
import trclib.TrcDiagnostics;

public class HighTalonErrorRateTest extends TrcDiagnostics.Test<Subsystem>
{
    private static final int ERROR_COUNT_THRESHOLD = 100;

    private final FrcCANTalon motor;

    public HighTalonErrorRateTest(String name, Subsystem subsystem, FrcCANTalon motor)
    {
        super(name, subsystem);
        this.motor = motor;
    }

    @Override
    public String runTest()
    {
        int errorCount = motor.getErrorCount();
        return errorCount > ERROR_COUNT_THRESHOLD?
            String.format("%s reported %d errors (> %d) - " +
                "there might be a problem with the CAN wiring.",
                this.getTestName(), errorCount, ERROR_COUNT_THRESHOLD): null;
    }

}

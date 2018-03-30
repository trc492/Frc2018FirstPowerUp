package team492.diagnostics.tests;

import team492.Robot;
import team492.diagnostics.DiagnosticsTest;
import team492.diagnostics.OnBoardDiagnostics.Subsystem;

public class PneumaticsCurrentlyLowTest extends DiagnosticsTest{

    private Robot robot;

    private static final double TARGET_PRESSURE_PSI = 80.0;

    public PneumaticsCurrentlyLowTest(String name, Robot robot){
        super(name, Subsystem.PRESSURE);
        this.robot = robot;
    }

    @Override
    public void test() {
        // No-op, result computed at reporting time.
    }

    @Override
    public TestResult getResult() {
        boolean pressureLow = robot.getPressure() < TARGET_PRESSURE_PSI;

        return new TestResult(pressureLow,
                "pneumatics pressure currently below " + TARGET_PRESSURE_PSI);
    }
}

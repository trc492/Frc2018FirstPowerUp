package team492.diagnostics;

import team492.OnBoardDiagnostics.Subsystem;
import team492.Robot;
import trclib.TrcDiagnostics;

public class PneumaticsCurrentlyLowTest extends TrcDiagnostics.Test<Subsystem>
{
    private Robot robot;

    private static final double TARGET_PRESSURE_PSI = 100.0;

    public PneumaticsCurrentlyLowTest(String name, Robot robot)
    {
        super(name, Subsystem.PNEUMATICS);
        this.robot = robot;
    }

    @Override
    public String runTest()
    {
        double pressure = robot.getPressure();
        return pressure < TARGET_PRESSURE_PSI?
            String.format("Pneumatics pressure too low, pressure=%.1f (< %.1f)", pressure, TARGET_PRESSURE_PSI): null;
    }

}

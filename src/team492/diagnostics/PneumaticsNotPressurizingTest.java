package team492.diagnostics;

import team492.OnBoardDiagnostics.Subsystem;
import team492.Robot;
import trclib.TrcDiagnostics;

public class PneumaticsNotPressurizingTest extends TrcDiagnostics.Test<Subsystem>
{
    private Robot robot;
    private boolean reachedWorkingPressure = false;

    private static final double WORKING_PRESSURE_PSI = 60.0;

    public PneumaticsNotPressurizingTest(String name, Robot robot)
    {
        super(name, Subsystem.PNEUMATICS);
        this.robot = robot;
    }

    @Override
    public String runTest()
    {
        double pressure = robot.getPressure();
        if (pressure >= WORKING_PRESSURE_PSI)
        {
            reachedWorkingPressure = true;
        }
        return reachedWorkingPressure? null:
            String.format(
                "Pneumatics never reached working pressure %.1f (< %.1f)", pressure, WORKING_PRESSURE_PSI);
    }

}

package team492.diagnostics;

import team492.OnBoardDiagnostics.Subsystem;
import team492.PixyVision;
import trclib.TrcDiagnostics;

public class PixyVisionTaskTerminatedTest extends TrcDiagnostics.Test<Subsystem>
{
    private final PixyVision pixyVision;

    private boolean hasTaskEverTerminated = false;

    public PixyVisionTaskTerminatedTest(String name, PixyVision pixyVision)
    {
        super(name, Subsystem.SENSORS);
        this.pixyVision = pixyVision;
    }

    @Override
    public String runTest()
    {
        hasTaskEverTerminated |= pixyVision.isTaskTerminatedAbnormally();
        return hasTaskEverTerminated?
            "Pixy vision background task terminated unexpectedly - " + "check console & Pixy camera wiring": null;
    }

}

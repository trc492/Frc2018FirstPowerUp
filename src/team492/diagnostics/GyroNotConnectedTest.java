package team492.diagnostics;

import frclib.FrcAHRSGyro;
import team492.OnBoardDiagnostics.Subsystem;
import trclib.TrcDiagnostics;

public class GyroNotConnectedTest extends TrcDiagnostics.Test<Subsystem>
{
    private FrcAHRSGyro gyro;

    private boolean hasLostConnection = false;

    public GyroNotConnectedTest(String name, FrcAHRSGyro gyro)
    {
        super(name, Subsystem.DRIVEBASE);
        this.gyro = gyro;
    }

    @Override
    public String runTest()
    {
        if (!gyro.ahrs.isConnected())
            hasLostConnection = true;
        return hasLostConnection? "Gyro lost connection during the match": null;
    }

}

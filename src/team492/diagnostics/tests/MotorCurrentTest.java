package team492.diagnostics.tests;

import frclib.FrcCANTalon;
import team492.diagnostics.OnBoardDiagnostics.Subsystem;
import team492.diagnostics.tests.ExpectSensorChangeTest;

public class MotorCurrentTest extends ExpectSensorChangeTest
{
    private static final double MIN_EXPECTED_CURRENT_CHANGE = 6.0; // 6A
    
    public MotorCurrentTest(String name, Subsystem subsystem, FrcCANTalon motor)
    {
        super(name, subsystem, motor.motor::getOutputCurrent, MIN_EXPECTED_CURRENT_CHANGE);
    }

    @Override
    public String getErrorMessage()
    {
        return super.getName() + " motor didn't receive enough current!";
    }
}

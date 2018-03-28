package team492.diagnostics.tests;

import team492.Robot;
import team492.diagnostics.DiagnosticsTest;

public class PneumaticsNotPressurizingTest extends DiagnosticsTest{

    private Robot robot;
    private boolean reachedWorkingPressure = false;

    private static final double WORKING_PRESSURE_PSI = 60.0;

    public PneumaticsNotPressurizingTest(String name, Robot robot){
        super(name);
        this.robot = robot;
    }

    @Override
    public void test() {
        if(robot.getPressure() >= WORKING_PRESSURE_PSI){
            reachedWorkingPressure = true;
        }
    }

    @Override
    public TestResult getResult() {
        return new TestResult(!reachedWorkingPressure,
                "pneumatics never reached working pressure");
    }
}

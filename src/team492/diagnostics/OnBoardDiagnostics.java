package team492.diagnostics;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import team492.Robot;
import team492.diagnostics.DiagnosticsTest.TestResult;
import team492.diagnostics.tests.DigitalSensorUnchangedTest;
import team492.diagnostics.tests.ElevatorPositionUnchangedTest;
import team492.diagnostics.tests.EncoderUnpluggedTest;
import team492.diagnostics.tests.GyroNotConnectedTest;
import team492.diagnostics.tests.UltrasonicUnpluggedTest;
import team492.diagnostics.tests.PixyVisionTaskTerminatedTest;
import team492.diagnostics.tests.HighTalonErrorRateTest;
import team492.diagnostics.tests.PneumaticsNotPressurizingTest;
import team492.diagnostics.tests.ElevatorLimitSwitchStuckTest;

public class OnBoardDiagnostics
{

    private Robot robot;
    private List<DiagnosticsTest> tests;

    public OnBoardDiagnostics(Robot robot)
    {
        this.robot = robot;
        tests = new ArrayList<>();
        tests.add(new EncoderUnpluggedTest(robot.leftFrontWheel, "left front encoder"));
        tests.add(new EncoderUnpluggedTest(robot.rightFrontWheel, "right front encoder"));
        tests.add(new EncoderUnpluggedTest(robot.leftRearWheel, "left rear encoder"));
        tests.add(new EncoderUnpluggedTest(robot.rightRearWheel, "right rear encoder"));

        tests.add(new HighTalonErrorRateTest(robot.leftFrontWheel, "left front wheel motor"));
        tests.add(new HighTalonErrorRateTest(robot.rightFrontWheel, "right front wheel motor"));
        tests.add(new HighTalonErrorRateTest(robot.leftRearWheel, "left rear wheel motor"));
        tests.add(new HighTalonErrorRateTest(robot.rightRearWheel, "right rear wheel motor"));
        tests.add(new HighTalonErrorRateTest(robot.elevator.elevatorMotor, "elevator motor"));

        tests.add(
            new ElevatorLimitSwitchStuckTest(robot.elevator, ElevatorLimitSwitchStuckTest.ElevatorLimitSwitch.LOWER));
        tests.add(
            new ElevatorLimitSwitchStuckTest(robot.elevator, ElevatorLimitSwitchStuckTest.ElevatorLimitSwitch.UPPER));

        if (robot.leftSonarArray != null || robot.leftSonarSensor != null)
        {
            tests.add(new UltrasonicUnpluggedTest(robot::getLeftSonarDistance, "left sonar"));
        }

        if (robot.rightSonarArray != null || robot.rightSonarSensor != null)
        {
            tests.add(new UltrasonicUnpluggedTest(robot::getRightSonarDistance, "right sonar"));
        }

        tests.add(new DigitalSensorUnchangedTest(robot.elevator.elevatorMotor::isLowerLimitSwitchActive,
            "elevator lower limit switch"));

        tests.add(new DigitalSensorUnchangedTest(robot.cubePickup::cubeInProximity, "grabber cube proximity sensor"));

        tests.add(new ElevatorPositionUnchangedTest("Elevator position", robot.elevator));

        tests.add(new PneumaticsNotPressurizingTest("Pneumatics", robot));

        if (robot.pixy != null)
        {
            tests.add(new PixyVisionTaskTerminatedTest("Pixy", robot.pixy));
        }

        if (robot.gyro != null)
        {
            tests.add(new GyroNotConnectedTest("Gyro", robot.gyro));
        }
    }

    public void doPeriodicTests()
    {
        tests.forEach(DiagnosticsTest::test);
    }
    
    public Map<String,Boolean> getDiagnosticResults()
    {
        Map<String,Boolean> map = new HashMap<>();
        for(DiagnosticsTest test:tests)
        {
            map.put(test.getName(), !test.getResult().faultDetected());
        }
        return map;
    }

    public void printDiagnostics()
    {
        boolean faults = false;

        for (DiagnosticsTest test : tests)
        {
            TestResult result = test.getResult();

            if (result.faultDetected())
            {
                faults = true;
                DriverStation.reportError("### Diagnostics: " + result.errorMessage(), false);
            }
        }

        if (faults)
        {
            robot.ledIndicator.indicateDiagnosticError();
        }
        else
        {
            DriverStation.reportError("### Diagnostics: No faults", false);
            robot.ledIndicator.indicateNoDiagnosticError();
        }
    }
}

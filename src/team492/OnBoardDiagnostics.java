package team492;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import hallib.HalDashboard;
import team492.diagnostics.ElevatorLimitSwitchStuckTest;
import team492.diagnostics.GyroNotConnectedTest;
import team492.diagnostics.HighTalonErrorRateTest;
import team492.diagnostics.PixyVisionTaskTerminatedTest;
import team492.diagnostics.PneumaticsCurrentlyLowTest;
import team492.diagnostics.PneumaticsNotPressurizingTest;
import trclib.TrcDiagnostics;
import trclib.TrcDiagnostics.Test;
import trclib.TrcTestAnalogSensorValueChange;
import trclib.TrcTestDigitalSensorValueChange;

public class OnBoardDiagnostics
{
    private static final String moduleName = "OnBoardDiagnostics";

    public enum Subsystem
    {
        DRIVEBASE,
        ELEVATOR,
        SENSORS,
        PNEUMATICS,
        GRABBER
    }

    private static final double EXPECTED_DRIVEBASE_ENCODER_CHANGE_INCHES = 10.0;
    private static final double EXPECTED_SONAR_CHANGE_INCHES = 6.0;
    private static final double EXPECTED_ELEVATOR_ENCODER_CHANGE_INCHES = 1.0;
    private static final double EXPECTED_GRABBER_CURRENT_CHANGE = 6.0; // 6A
    private static final String errMsgAnalogValueChange =
        "Value did not change enough, could be defective or disconnected.";
    private static final String errMsgDigitalStateChange =
        "State did not change, could be defective or disconnected.";

    private Robot robot;
    private TrcDiagnostics<Subsystem> testCollection = new TrcDiagnostics<>();

    public OnBoardDiagnostics(Robot robot)
    {
        this.robot = robot;
        //
        // DriveBase
        //
        testCollection.addTest(new TrcTestAnalogSensorValueChange<Subsystem>(
            "lfEncoder", Subsystem.DRIVEBASE, robot.leftFrontWheel::getPosition,
            EXPECTED_DRIVEBASE_ENCODER_CHANGE_INCHES, errMsgAnalogValueChange));
        testCollection.addTest(new TrcTestAnalogSensorValueChange<Subsystem>(
            "rfEncoder", Subsystem.DRIVEBASE, robot.rightFrontWheel::getPosition,
            EXPECTED_DRIVEBASE_ENCODER_CHANGE_INCHES, errMsgAnalogValueChange));
        testCollection.addTest(new TrcTestAnalogSensorValueChange<Subsystem>(
            "lrEncoder", Subsystem.DRIVEBASE, robot.leftRearWheel::getPosition,
            EXPECTED_DRIVEBASE_ENCODER_CHANGE_INCHES, errMsgAnalogValueChange));
        testCollection.addTest(new TrcTestAnalogSensorValueChange<Subsystem>(
            "rrEncoder", Subsystem.DRIVEBASE, robot.rightRearWheel::getPosition,
            EXPECTED_DRIVEBASE_ENCODER_CHANGE_INCHES, errMsgAnalogValueChange));
        testCollection.addTest(
            new HighTalonErrorRateTest("lfMotorErrors", Subsystem.DRIVEBASE, robot.leftFrontWheel));
        testCollection.addTest(
            new HighTalonErrorRateTest("rfMotorErrors", Subsystem.DRIVEBASE, robot.rightFrontWheel));
        testCollection.addTest(
            new HighTalonErrorRateTest("lrMotorErrors", Subsystem.DRIVEBASE, robot.leftRearWheel));
        testCollection.addTest(
            new HighTalonErrorRateTest("rrMotorErrors", Subsystem.DRIVEBASE, robot.rightRearWheel));
        if (robot.gyro != null)
        {
            testCollection.addTest(new GyroNotConnectedTest("Gyro connected", robot.gyro));
        }
        //
        // Elevator
        //
        testCollection.addTest(new TrcTestAnalogSensorValueChange<Subsystem>(
            "elevatorEncoder", Subsystem.ELEVATOR, robot.elevator::getPosition,
            EXPECTED_ELEVATOR_ENCODER_CHANGE_INCHES, errMsgAnalogValueChange));
        testCollection.addTest(
            new HighTalonErrorRateTest("elevatorMotorErrors", Subsystem.ELEVATOR, robot.elevator.elevatorMotor));
        //CodeReview: Aren't ElevatorLimitSwitch tests redundant since you have the StuckTest already?
        testCollection.addTest(new TrcTestDigitalSensorValueChange<Subsystem>(
            "lowerElevatorLimitSwitch", Subsystem.ELEVATOR, robot.elevator.elevatorMotor::isLowerLimitSwitchActive,
            errMsgDigitalStateChange));
        testCollection.addTest(new TrcTestDigitalSensorValueChange<Subsystem>(
            "upperElevatorLimitSwitch", Subsystem.ELEVATOR, robot.elevator.elevatorMotor::isUpperLimitSwitchActive,
            errMsgDigitalStateChange));
        testCollection.addTest(
            new ElevatorLimitSwitchStuckTest(robot.elevator, ElevatorLimitSwitchStuckTest.ElevatorLimitSwitch.LOWER));
        testCollection.addTest(
            new ElevatorLimitSwitchStuckTest(robot.elevator, ElevatorLimitSwitchStuckTest.ElevatorLimitSwitch.UPPER));
        //
        // Sensors
        //
        if (robot.leftSonarArray != null || robot.leftSonarSensor != null)
        {
            testCollection.addTest(new TrcTestAnalogSensorValueChange<Subsystem>(
                "leftSonar", Subsystem.SENSORS, robot::getLeftSonarDistance,
                EXPECTED_SONAR_CHANGE_INCHES, errMsgAnalogValueChange));
        }

        if (robot.rightSonarArray != null || robot.rightSonarSensor != null)
        {
            testCollection.addTest(new TrcTestAnalogSensorValueChange<Subsystem>(
                "rightSonar", Subsystem.SENSORS, robot::getRightSonarDistance,
                EXPECTED_SONAR_CHANGE_INCHES, errMsgAnalogValueChange));
        }

        if (robot.pixy != null)
        {
            testCollection.addTest(new PixyVisionTaskTerminatedTest("Pixy errors", robot.pixy));
        }

        testCollection.addTest(new TrcTestDigitalSensorValueChange<Subsystem>(
            "cubeProximitySensor", Subsystem.SENSORS, robot.cubePickup::cubeInProximity, errMsgDigitalStateChange));
        //
        // Pneumatics
        //
        testCollection.addTest(new PneumaticsNotPressurizingTest("pneumaticsRegulatedPressure", robot));
        testCollection.addTest(new PneumaticsCurrentlyLowTest("pneumaticsTankPressure", robot));
        //
        // Grabber
        //
        testCollection.addTest(new TrcTestAnalogSensorValueChange<Subsystem>(
            "leftGrabber", Subsystem.GRABBER, robot.cubePickup.controlMotor.motor::getOutputCurrent,
            EXPECTED_GRABBER_CURRENT_CHANGE, errMsgAnalogValueChange));
        testCollection.addTest(new TrcTestAnalogSensorValueChange<Subsystem>(
            "rightGrabber", Subsystem.GRABBER, robot.cubePickup.slaveMotor.motor::getOutputCurrent,
            EXPECTED_GRABBER_CURRENT_CHANGE, errMsgAnalogValueChange));
    }

    public void updateDiagnosticsAndDashboard()
    {
        testCollection.runAllTests();

        for (Test<Subsystem> test: testCollection)
        {
            HalDashboard.putBoolean("Diagnostics/" + test.getTestName(), test.hasPassed());
        }

        Map<Subsystem, Boolean> subsystemResults = testCollection.getTestGroupResults();
        for(Map.Entry<Subsystem, Boolean> entry: subsystemResults.entrySet())
        {
            HalDashboard.putBoolean("Test/" + entry.getKey().name(), entry.getValue());
        }
    }

    public void printDiagnostics()
    {
        boolean faults = false;

        for (Test<Subsystem> test: testCollection)
        {
            if (!test.hasPassed())
            {
                String errMsg = test.getTestName() + ": " + test.getTestError();
                DriverStation.reportError(errMsg, false);
                robot.globalTracer.traceErr(moduleName, errMsg);
                faults = true;
            }
        }

        if (faults)
        {
            robot.ledIndicator.indicateDiagnosticError();
        }
        else
        {
            robot.ledIndicator.indicateNoDiagnosticError();
            String message = "### Diagnostics: No faults";
            DriverStation.reportError(message, false);
            robot.globalTracer.traceInfo(moduleName, message);
        }
    }

}

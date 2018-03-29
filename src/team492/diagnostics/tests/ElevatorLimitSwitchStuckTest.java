package team492.diagnostics.tests;

import team492.Elevator;
import team492.diagnostics.DiagnosticsTest;
import team492.diagnostics.OnBoardDiagnostics.Subsystem;

import java.util.function.Supplier;


public class ElevatorLimitSwitchStuckTest extends DiagnosticsTest {

    private static final double ELEVATOR_POSITION_CHANGE_THRESHOLD_INCHES = 2.0;

    public enum ElevatorLimitSwitch {
        LOWER,
        UPPER
    }

    private final Supplier<Boolean> chosenLimitSwitch;
    private final ElevatorLimitSwitch chosenLimitSwitchEnum;
    private final Elevator elevator;

    private boolean limitSwitchEverStuck = false;

    private Double elevatorPosWhenLimitSwitchActivatedInches = null;

    public ElevatorLimitSwitchStuckTest(Elevator elevator, ElevatorLimitSwitch chosenLimitSwitchEnum) {
        super(chosenLimitSwitchEnum.name() + " elevator stuck", Subsystem.ELEVATOR);
        if (chosenLimitSwitchEnum == ElevatorLimitSwitch.LOWER) {
            this.chosenLimitSwitch = elevator.elevatorMotor::isLowerLimitSwitchActive;
        } else {
            this.chosenLimitSwitch = elevator.elevatorMotor::isUpperLimitSwitchActive;
        }

        this.elevator = elevator;
        this.chosenLimitSwitchEnum = chosenLimitSwitchEnum;
    }

    @Override
    public void test() {
        boolean limitSwitchActive = chosenLimitSwitch.get();

        // Save current position when limit switch is first activated
        if (limitSwitchActive && elevatorPosWhenLimitSwitchActivatedInches == null) {
            elevatorPosWhenLimitSwitchActivatedInches = elevator.getPosition();
        }

        if (elevatorPosWhenLimitSwitchActivatedInches != null) {
            final double changeInElevatorPositionInches =
                    Math.abs(elevator.getPosition() - elevatorPosWhenLimitSwitchActivatedInches);

            // If limit switch is still pressed and we moved more than we should have,
            // consider limit switch stuck
            if (limitSwitchActive &&
                    changeInElevatorPositionInches > ELEVATOR_POSITION_CHANGE_THRESHOLD_INCHES) {
                limitSwitchEverStuck = true;
            }

            // Once the limit switch resets, clear the elevator position for next time
            if (!limitSwitchActive) {
                elevatorPosWhenLimitSwitchActivatedInches = null;
            }
        }
    }

    @Override
    public TestResult getResult() {
        return new TestResult(limitSwitchEverStuck,
                "elevator " + chosenLimitSwitchEnum.name() +
                        " limit switch got stuck at least once");
    }
}

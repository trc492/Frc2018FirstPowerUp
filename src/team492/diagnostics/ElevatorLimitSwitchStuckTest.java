package team492.diagnostics;

import java.util.function.Supplier;

import team492.Elevator;
import team492.OnBoardDiagnostics.Subsystem;
import trclib.TrcDiagnostics;

public class ElevatorLimitSwitchStuckTest extends TrcDiagnostics.Test<Subsystem>
{
    private static final double ELEVATOR_POSITION_CHANGE_THRESHOLD_INCHES = 2.0;

    public enum ElevatorLimitSwitch
    {
        LOWER, UPPER
    }

    private final Supplier<Boolean> chosenLimitSwitch;
    private final ElevatorLimitSwitch chosenLimitSwitchEnum;
    private final Elevator elevator;

    private boolean limitSwitchEverStuck = false;

    private Double elevatorPosWhenLimitSwitchActivatedInches = null;

    public ElevatorLimitSwitchStuckTest(Elevator elevator, ElevatorLimitSwitch chosenLimitSwitchEnum)
    {
        super("elevator" + chosenLimitSwitchEnum.name() + "LimitSwitch", Subsystem.ELEVATOR);
        if (chosenLimitSwitchEnum == ElevatorLimitSwitch.LOWER)
        {
            this.chosenLimitSwitch = elevator.elevatorMotor::isLowerLimitSwitchActive;
        }
        else
        {
            this.chosenLimitSwitch = elevator.elevatorMotor::isUpperLimitSwitchActive;
        }

        this.elevator = elevator;
        this.chosenLimitSwitchEnum = chosenLimitSwitchEnum;
    }

    @Override
    public String runTest()
    {
        boolean limitSwitchActive = chosenLimitSwitch.get();

        // Save current position when limit switch is first activated
        if (limitSwitchActive && elevatorPosWhenLimitSwitchActivatedInches == null)
        {
            elevatorPosWhenLimitSwitchActivatedInches = elevator.getPosition();
        }

        if (elevatorPosWhenLimitSwitchActivatedInches != null)
        {
            final double changeInElevatorPositionInches =
                Math.abs(elevator.getPosition() - elevatorPosWhenLimitSwitchActivatedInches);

            // If limit switch is still pressed and we moved more than we should
            // have,
            // consider limit switch stuck
            if (limitSwitchActive && changeInElevatorPositionInches > ELEVATOR_POSITION_CHANGE_THRESHOLD_INCHES)
            {
                limitSwitchEverStuck = true;
            }

            // Once the limit switch resets, clear the elevator position for
            // next time
            if (!limitSwitchActive)
            {
                elevatorPosWhenLimitSwitchActivatedInches = null;
            }
        }

        return limitSwitchEverStuck?
            "elevator" + chosenLimitSwitchEnum.name() + "LimitSwitch got stuck at least once": null;
    }

}

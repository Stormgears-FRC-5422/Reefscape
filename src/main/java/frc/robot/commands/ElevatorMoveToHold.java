package frc.robot.commands;

import frc.robot.subsystems.Elevator;

// This is a specialization of the MoveToPosition command that
// has a constructor allowing it to hold at the current position when created
// If the position passed in is a level, it is equivalent to
// move-to that level and then hold position, rather than end-then-release.
public class ElevatorMoveToHold extends ElevatorMoveToPosition {

    /**
     * Default Constructor
     * Holds elevator at the current position
     * @param  elevatorSubsystem  Elevator Subsystem
     */
    public ElevatorMoveToHold(Elevator elevatorSubsystem) {
        super(elevatorSubsystem, elevatorSubsystem.getCurrentPosition());
    }

    /**
     * Overloaded Constructor
     * Holds elevator at the specified position
     * @param  elevatorSubsystem Elevator Subsystem
     * @param  level position to hold the elevator at
     */
    public ElevatorMoveToHold(Elevator elevatorSubsystem, Elevator.ElevatorLevel level) {
        super(elevatorSubsystem, level);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

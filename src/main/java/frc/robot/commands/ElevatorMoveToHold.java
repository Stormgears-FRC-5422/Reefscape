package frc.robot.commands;

import frc.robot.subsystems.Elevator;

// This is a specialization of the MoveToPosition command that
// has a constructor allowing it to hold at the current position when created
// If the position passed in is a level, it is equivalent to
// move-to that level and then hold position, rather than end-then-release.
public class ElevatorMoveToHold extends ElevatorMoveToPosition {

    public ElevatorMoveToHold(Elevator elevatorSubsystem) {
        super(elevatorSubsystem, elevatorSubsystem.getCurrentPosition());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

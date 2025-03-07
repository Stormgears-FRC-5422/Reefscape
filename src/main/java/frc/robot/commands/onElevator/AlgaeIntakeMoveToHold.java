package frc.robot.commands.onElevator;

import frc.robot.subsystems.onElevator.AlgaeIntake;
import frc.robot.subsystems.onElevator.Elevator;

// This is a specialization of the MoveToPosition command that
// has a constructor allowing it to hold at the current position when created
// If the position passed in is a level, it is equivalent to
// move-to that level and then hold position, rather than end-then-release.
public class AlgaeIntakeMoveToHold extends AlgaeIntakeMoveToPosition {

    public AlgaeIntakeMoveToHold(AlgaeIntake algaeIntake) {
        super(algaeIntake, algaeIntake.getCurrentPosition());
    }

    public AlgaeIntakeMoveToHold(AlgaeIntake algaeIntake, AlgaeIntake.IntakeTarget target) {
        super(algaeIntake, target);
    }

    public AlgaeIntakeMoveToHold(AlgaeIntake algaeIntake, double position) {
        super(algaeIntake, position);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

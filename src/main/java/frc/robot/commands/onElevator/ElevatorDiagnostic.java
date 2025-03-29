package frc.robot.commands.onElevator;

import frc.robot.subsystems.onElevator.Elevator;
import frc.robot.subsystems.onElevator.Elevator.ElevatorLevel;
import frc.utils.StormCommand;

public class ElevatorDiagnostic extends StormCommand {
    private boolean skip;

    private Elevator elevator;
    private boolean up;

    public ElevatorDiagnostic(Elevator elevator, boolean up) {
        this.elevator = elevator;
        this.up = up;

        skip = safeAddRequirements(elevator);
    }

    @Override
    public void initialize() {
        if (skip){
            return;
        }
        elevator.setTargetLevel(up ? ElevatorLevel.CEILING : ElevatorLevel.FLOOR);
        elevator.setState(Elevator.ElevatorState.SIMPLE_MOTION);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return skip;
    }

    @Override
    public void end(boolean interrupted) {
        if (!skip){
            elevator.setState(Elevator.ElevatorState.IDLE);
        } else if (elevator != null){
            elevator.setState(Elevator.ElevatorState.UNKNOWN);
        }
        super.end(interrupted);
    }
}


package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.utils.StormCommand;

public class ElevatorDiagnostic extends StormCommand {

    private Elevator elevator;
    private boolean up;

    public ElevatorDiagnostic(Elevator elevator, boolean up) {
        this.elevator = elevator;
        this.addRequirements(elevator);
        this.up = up;
    }

    @Override
    public void initialize() {
        elevator.setTargetLevel(up ? ElevatorLevel.CEILING : ElevatorLevel.FLOOR);
        elevator.setState(Elevator.ElevatorState.SIMPLE_MOTION);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setState(Elevator.ElevatorState.IDLE);
        super.end(interrupted);
    }
}


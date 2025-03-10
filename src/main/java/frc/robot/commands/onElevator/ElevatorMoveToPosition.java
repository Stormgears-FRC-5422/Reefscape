package frc.robot.commands.onElevator;

import frc.robot.subsystems.onElevator.Elevator;
import frc.robot.subsystems.onElevator.Elevator.ElevatorLevel;
import frc.utils.StormCommand;

public class ElevatorMoveToPosition extends StormCommand {
    protected final Elevator elevatorSubsystem;
    protected final double targetPosition;

    public ElevatorMoveToPosition(Elevator elevatorSubsystem, double position) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetPosition = position;

        if (elevatorSubsystem != null) {
            addRequirements(elevatorSubsystem);
        }
    }

    public ElevatorMoveToPosition(Elevator elevatorSubsystem, ElevatorLevel targetLevel) {
        this(elevatorSubsystem, targetLevel.getValue());
    }

    @Override
    public void initialize() {
        super.initialize();
        elevatorSubsystem.setTargetPosition(targetPosition);
        elevatorSubsystem.setState(Elevator.ElevatorState.PID_MOTION);
    }

    @Override
    public void execute() {
        super.execute();
        // moving is done in the subsystem
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setState(Elevator.ElevatorState.IDLE);
        super.end(interrupted);
    }
}

package frc.robot.commands.onElevator;

import frc.robot.subsystems.onElevator.Elevator;
import frc.robot.subsystems.onElevator.Elevator.ElevatorLevel;
import frc.utils.StormCommand;

public class ElevatorMoveToPosition extends StormCommand {
    private boolean skip;
    protected final Elevator elevatorSubsystem;
    protected final double targetPosition;

    public ElevatorMoveToPosition(Elevator elevatorSubsystem, double position) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetPosition = position;

        skip = safeAddRequirements(elevatorSubsystem);
    }

    public ElevatorMoveToPosition(Elevator elevatorSubsystem, ElevatorLevel targetLevel) {
        this(elevatorSubsystem, targetLevel.getValue());
    }

    @Override
    public void initialize() {
        super.initialize();
        if (skip){
            console("Elevator:" + skip);
            return;
        }
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
        return skip || elevatorSubsystem.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        if (!skip){
            elevatorSubsystem.setState(Elevator.ElevatorState.IDLE);
        } else if (elevatorSubsystem != null){
            elevatorSubsystem.setState(Elevator.ElevatorState.UNKNOWN);
        }
        super.end(interrupted);
    }
}

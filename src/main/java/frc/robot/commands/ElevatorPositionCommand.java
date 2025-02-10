package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.utils.StormCommand;

public class ElevatorPositionCommand extends StormCommand {
    private final Elevator elevatorSubsystem;
    private final ElevatorLevel targetLevel;

    public ElevatorPositionCommand(Elevator elevatorSubsystem, ElevatorLevel targetLevel) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetLevel = targetLevel;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        elevatorSubsystem.setTargetLevel(targetLevel);
        elevatorSubsystem.setState(Elevator.ElevatorState.PID_MOTION);
    }

    @Override
    public void execute() {
        super.execute();
        // moving is done in the subsystem
    }

    @Override
    public boolean isFinished() {
        // return elevatorSubsystem.getTargetLevel() == targetLevel;
        // For now this doesn't really end - we want it to hold. This might not need a separate command
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setState(Elevator.ElevatorState.IDLE);
        super.end(interrupted);
    }
}

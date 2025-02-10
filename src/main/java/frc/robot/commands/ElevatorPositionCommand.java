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
    }

    @Override
    public void execute() {
        // moving is done in the subsystem
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.getTargetLevel() == targetLevel;
    }
}

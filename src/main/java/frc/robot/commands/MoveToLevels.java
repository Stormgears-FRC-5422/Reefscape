package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorLevels;
import frc.utils.StormCommand;

public class MoveToLevels extends StormCommand {

    private final Elevator elevatorSubsystem;
    private final ElevatorLevels targetLevel;

    public MoveToLevels(Elevator elevatorSubsystem, ElevatorLevels targetLevel) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetLevel = targetLevel;
        
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        elevatorSubsystem.setElevatorLevel(targetLevel);
    }

    @Override
    public void execute() {
        // moving is ddone in the subsystem
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.getElevatorLevel() == targetLevel;
    }
}

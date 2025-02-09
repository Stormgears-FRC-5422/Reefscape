package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.utils.StormCommand;

public class HomeElevator extends StormCommand {
    private final Elevator elevator;

    public HomeElevator(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
        elevator.setCurrentState(ElevatorState.HOMING);
        // elevator.setTargetLevel(ElevatorLevel.FLOOR);
        // elevator.setSpeed(0.05);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtHome();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            elevator.setCurrentState(ElevatorState.HOME);
        } else {
            elevator.setCurrentState(ElevatorState.UNKNOWN);
        }

        super.end(interrupted);
    }
}

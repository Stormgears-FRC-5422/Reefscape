package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.utils.StormCommand;

public class ElevatorHome extends StormCommand {
    private final Elevator elevator;

    public ElevatorHome(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
        elevator.setState(ElevatorState.HOMING);
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
            elevator.setState(ElevatorState.HOME);
        } else {
            elevator.setState(ElevatorState.UNKNOWN);
        }

        super.end(interrupted);
    }
}

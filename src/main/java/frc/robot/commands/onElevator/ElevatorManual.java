package frc.robot.commands.onElevator;

import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.onElevator.Elevator;
import frc.robot.subsystems.onElevator.Elevator.ElevatorLevel;
import frc.utils.StormCommand;

public class ElevatorManual extends StormCommand {
    private Elevator elevator;
    private ReefscapeJoystick buttonBoard;
    public ElevatorManual(Elevator elevator, ReefscapeJoystick buttonBoard) {
        this.elevator = elevator;
        this.buttonBoard = buttonBoard;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
        elevator.setTargetPosition(elevator.getCurrentPosition());
        elevator.setState(Elevator.ElevatorState.PID_MOTION);
    }

    @Override
    public void execute() {
        super.execute();

        // Note that this has the potential to drift a bit over time,
        // but this approach allows us to avoid backlash when changing direction
        // or stopping.
        double currentPosition = elevator.getCurrentPosition();

        Direction direction = buttonBoard.elevatorUp() ? Direction.UP
            : buttonBoard.elevatorDown() ? Direction.DOWN
            : Direction.HOLD;

//        console("direction = " + direction + ", currentPosition = " + currentPosition);
        if (direction == Direction.UP) {
            elevator.setTargetLevel(ElevatorLevel.TOP);
        } else if (direction == Direction.DOWN) {
            elevator.setTargetLevel(ElevatorLevel.LEVEL1);
        } else {
            elevator.setTargetPosition(currentPosition);
        }
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

    enum Direction {
        HOLD,
        UP,
        DOWN
    }
}

package frc.robot.commands.onElevator;

import frc.robot.Constants;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.onElevator.Elevator;
import frc.robot.subsystems.onElevator.Elevator.ElevatorLevel;
import frc.utils.StormCommand;

public class ElevatorManual extends StormCommand {
    private boolean skip;
    private Elevator elevator;
    private ReefscapeJoystick buttonBoard;
    public ElevatorManual(Elevator elevator, ReefscapeJoystick buttonBoard) {
        this.elevator = elevator;
        this.buttonBoard = buttonBoard;
        skip = safeAddRequirements(elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (skip){
            return;
        }
        elevator.setTargetPosition(elevator.getCurrentPosition());
        if (Constants.Elevator.useSimpleMotion) {
            elevator.setState(Elevator.ElevatorState.SIMPLE_MOTION);
        } else {
            elevator.setState(Elevator.ElevatorState.PID_MOTION);
        }
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
            elevator.setTargetLevel(ElevatorLevel.BOTTOM);
        } else {
            elevator.setTargetPosition(currentPosition);
        }
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

    enum Direction {
        HOLD,
        UP,
        DOWN
    }
}

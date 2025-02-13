package frc.robot.commands;

import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.utils.StormCommand;

public class ElevatorManual extends StormCommand {
    enum Direction {
        UNKNOWN,
        HOLD,
        IDLE,
        UP,
        DOWN
    }

    private Elevator elevator;
    private ReefscapeJoystick buttonBoard;
    private Direction oldDirection;

    public ElevatorManual(Elevator elevator, ReefscapeJoystick buttonBoard) {
        this.elevator = elevator;
        this.buttonBoard = buttonBoard;
        this.addRequirements(elevator);
    }

    @Override
    public void initialize() {
        oldDirection = Direction.UNKNOWN;
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        Direction direction = buttonBoard.elevatorUp() ? Direction.UP :
            buttonBoard.elevatorDown() ? Direction.DOWN : Direction.HOLD;

        // TODO - use coral sensor to decide?
        boolean holdWhenStopped = true;
        if (direction == Direction.HOLD && holdWhenStopped == false) {
            direction = Direction.IDLE;
        }

        console("direction = " + direction + ", oldDirection = " + oldDirection, 50);
        if (direction == oldDirection) {
            return;
        }

        if (direction == Direction.UP) {
            elevator.setTargetLevel(ElevatorLevel.CEILING);
            elevator.setState(Elevator.ElevatorState.SIMPLE_MOTION);
        } else if (direction == Direction.DOWN) {
            elevator.setTargetLevel(ElevatorLevel.FLOOR);
            elevator.setState(Elevator.ElevatorState.SIMPLE_MOTION);
        } else if (holdWhenStopped) {
            elevator.setTargetPosition(elevator.getCurrentPosition());
            elevator.setState(Elevator.ElevatorState.PID_MOTION);
        } else {
            elevator.setState(Elevator.ElevatorState.IDLE);
        }

        // remember what we were doing
        oldDirection = direction;
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

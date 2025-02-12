package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.utils.StormCommand;

public class ElevatorHome extends StormCommand {
    // skip is static in case multiple instances of this command get created
    private static boolean skip;

    private final Elevator elevator;
    private RobotState robotState;
    // Don't allow homing if already homed. At least for now.
    // At this point the home will happen automatically at the beginning of auto or teleop
    // whichever is run first.

    public ElevatorHome(Elevator elevator) {
        this.elevator = elevator;
        robotState = RobotState.getInstance();
        skip = false;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (!robotState.elevatorHasBeenHomed()) {
            elevator.setState(ElevatorState.HOMING);
        } else {
            skip = true;
            console("skipping elevator home - already homed");
        }
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return skip || elevator.isAtHome();
    }

    @Override
    public void end(boolean interrupted) {
        if (!skip) {
            if (!interrupted) {
                elevator.setState(ElevatorState.HOME);
            } else {
                elevator.setState(ElevatorState.UNKNOWN);
            }
        }

        super.end(interrupted);
    }
}

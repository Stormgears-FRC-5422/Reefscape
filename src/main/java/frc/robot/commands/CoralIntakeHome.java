package frc.robot.commands;

import frc.robot.RobotState;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.utils.StormCommand;

import static java.util.Objects.isNull;

public class CoralIntakeHome extends StormCommand {
    private boolean skip;

    private final CoralIntake coralIntake;
    private RobotState robotState;
    // Don't allow homing if already homed. At least for now.
    // At this point the home will happen automatically at the beginning of auto or teleop
    // whichever is run first.

    public CoralIntakeHome(CoralIntake coralIntake) {
        this.coralIntake = coralIntake;
        robotState = RobotState.getInstance();

        if (!isNull(coralIntake)) {
            addRequirements(coralIntake);
        }
    }

    @Override
    public void initialize() {
        super.initialize();
        skip = robotState.getIntakeWristHasBeenHomed();
        if (!skip) {
            coralIntake.setState(CoralIntake.CoralIntakeState.HOMING);
        } else {
            console("skipping coralIntake home - already homed");
        }
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return skip || coralIntake.isAtHome();
    }

    @Override
    public void end(boolean interrupted) {
        if (!skip) {
            if (!interrupted) {
                coralIntake.setState(CoralIntake.CoralIntakeState.HOME);
            } else {
                coralIntake.setState(CoralIntake.CoralIntakeState.UNKNOWN);
            }
        }

        super.end(interrupted);
    }
}

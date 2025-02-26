package frc.robot.commands.onElevator;

import frc.robot.RobotState;
import frc.robot.subsystems.onElevator.AlgaeIntake;
import frc.utils.StormCommand;

import static java.util.Objects.isNull;

public class AlgaeIntakeHome extends StormCommand {
    private boolean skip;

    private final AlgaeIntake intake;
    private RobotState robotState;
    // Don't allow homing if already homed. At least for now.
    // At this point the home will happen automatically at the beginning of auto or teleop
    // whichever is run first.

    public AlgaeIntakeHome(AlgaeIntake intake) {
        this.intake = intake;
        robotState = RobotState.getInstance();

        if (!isNull(intake)) {
            addRequirements(intake);
        }
    }

    @Override
    public void initialize() {
        super.initialize();
        skip = robotState.getIntakeWristHasBeenHomed();
        if (!skip) {
            intake.setState(AlgaeIntake.IntakeState.HOMING);
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
        return skip || intake.isAtHome();
    }

    @Override
    public void end(boolean interrupted) {
        if (!skip) {
            if (!interrupted) {
                intake.setState(AlgaeIntake.IntakeState.HOME);
            } else {
                intake.setState(AlgaeIntake.IntakeState.UNKNOWN);
            }
        }

        super.end(interrupted);
    }
}

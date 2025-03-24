package frc.robot.commands.onElevator;
import frc.robot.RobotState;
import frc.robot.subsystems.onElevator.AlgaeIntake;
import frc.robot.subsystems.onElevator.AlgaeIntake.IntakeState;
import frc.utils.StormCommand;
public class AlgaeIntakeHome extends StormCommand {
    private final AlgaeIntake intake;
    private boolean skip;
    // Don't allow homing if already homed. At least for now.
    // At this point the home will happen automatically at the beginning of auto or teleop
    // whichever is run first.
    public AlgaeIntakeHome(AlgaeIntake intake) {
        this.intake = intake;
        skip = intake.getAlgaeIsAtHome();
        if (intake != null) {
            addRequirements(intake);
        }
    }
    @Override
    public void initialize() {
        super.initialize();
        intake.setState(IntakeState.HOMING);
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
        if(skip) {
            intake.setState(AlgaeIntake.IntakeState.UNKNOWN);
        } else {
            intake.setState(AlgaeIntake.IntakeState.HOME);
        }
        super.end(interrupted);
    }
}


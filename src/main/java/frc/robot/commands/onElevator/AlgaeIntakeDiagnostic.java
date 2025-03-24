package frc.robot.commands.onElevator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.onElevator.AlgaeIntake;
import frc.utils.StormCommand;
import frc.robot.Constants;
import frc.robot.Constants.SparkConstants;
import frc.robot.subsystems.onElevator.AlgaeIntake.IntakeState;
public class AlgaeIntakeDiagnostic extends StormCommand {
    public AlgaeIntake algaeIntake;
    private boolean up;
    public AlgaeIntakeDiagnostic(AlgaeIntake algaeIntake, boolean up) {
        this.algaeIntake = algaeIntake;
        this.up = up;
        if (algaeIntake != null) {
            addRequirements(algaeIntake);
        }
    }
    @Override
    public void initialize() {
        algaeIntake.setState(up ? IntakeState.UP : IntakeState.DOWN);
    }
    @Override
    public void execute() {
        super.execute();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        algaeIntake.setState(IntakeState.UNKNOWN);
    }
}

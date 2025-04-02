package frc.robot.commands.onElevator;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.onElevator.CoralIntake;
import frc.utils.StormCommand;

public class CoralHoldForAlgae extends StormCommand {
    private boolean skip;
    CoralIntake coralIntake;
    public CoralHoldForAlgae(CoralIntake coralIntake) {
        this.coralIntake = coralIntake;
        skip = safeAddRequirements(coralIntake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
        if (skip){
            return;
        }
        coralIntake.setState(CoralIntake.IntakeState.HOLD_FOR_ALGAE);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        super.execute();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!skip){
            coralIntake.setState(CoralIntake.IntakeState.IDLE);
        } else if (coralIntake != null){
            coralIntake.setState(CoralIntake.IntakeState.UNKNOWN);
        }
        super.end(interrupted);
    }
}

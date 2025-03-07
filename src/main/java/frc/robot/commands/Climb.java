package frc.robot.commands;

import frc.robot.RobotState;
import frc.robot.subsystems.Climber;
import frc.utils.StormCommand;

public class Climb extends StormCommand {
    Climber climber;
    RobotState state;
    boolean forward = false;

    public Climb(Climber c, boolean forward) {
        climber = c;
        this.forward = forward;
        this.state = RobotState.getInstance();
        addRequirements(c);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (forward) {
            climber.setState(Climber.ClimberState.PULLING);
        }else{
            climber.setState(Climber.ClimberState.PUSHING);
        }
    }

    @Override
    public boolean isFinished() {
        if (climber.isLockedIn()) {
            console("locked in. isFinished = true");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            climber.setState(Climber.ClimberState.IDLE);
        } else {
            climber.setState(Climber.ClimberState.HANGING);
        }
        super.end(interrupted);
    }

}


package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Climber;
import frc.utils.StormCommand;

public class Climb extends StormCommand {
    Climber climber;
    RobotState state;
    boolean forward = false;
    int count = 0;
    boolean isTriggered;

    public Climb(Climber c, boolean forward) {
        climber = c;
        this.forward = forward;
        this.state = RobotState.getInstance();
        addRequirements(c);
    }

    @Override
    public void initialize() {
        isTriggered = false;
        count = 0;
        super.initialize();
        if (forward) {
            climber.setState(Climber.ClimberState.PULLING);
        } else {
            climber.setState(Climber.ClimberState.PUSHING);
        }
    }

    @Override
    public void execute() {
        if(RobotState.getInstance().isClimberLockedIn()){
            isTriggered = RobotState.getInstance().isClimberLockedIn();
        }
        if (forward) {
            if (isTriggered) {
                climber.setState(Climber.ClimberState.PUSHING);
                count++;
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (isTriggered) {
            return count > Constants.Climber.timeOut;
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


package frc.robot.commands;

import frc.utils.StormCommand;

// TODO: Code this after week 0
//  Align with Processor -> Outtake Algae
public class AutoProcessorCommand extends StormCommand {
    int counter = 0;

    public AutoProcessorCommand() {
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        console("AutoProcessorCommand executed");
        counter++;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return counter>0;
    }
}

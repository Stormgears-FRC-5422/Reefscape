package frc.robot.commands;

import frc.robot.Constants;
import frc.utils.StormCommand;

// TODO: Code this (waiting for Krish's code)
//  Align with station -> Raise Elevator Level -> Set Arm Position -> Intake Coral
public class AutoStationCommand extends StormCommand  {
    int counter = 0;

    public AutoStationCommand() {
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        console("AutoStationCommand executed");
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

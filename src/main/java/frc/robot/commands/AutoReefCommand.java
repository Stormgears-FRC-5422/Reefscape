package frc.robot.commands;

import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.utils.StormCommand;

// TODO: Code this (waiting for Krish's code)
//  Align with Reef -> Raise Elevator Level -> Set Arm Position -> Outtake Coral
public class AutoReefCommand extends StormCommand {
    int counter = 0;
    ElevatorLevel level = ElevatorLevel.LEVEL4;
    Boolean isRightReef = false;

    public AutoReefCommand(ElevatorLevel level, Boolean isRightReef) {
        this.level = level;
        this.isRightReef = isRightReef;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        console("AutoReefCommand executed");
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


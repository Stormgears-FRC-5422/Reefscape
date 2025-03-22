package frc.robot.commands;

import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;

public class DummyDriveCommand extends StormCommand {
    DrivetrainBase drivetrainBase;
    public DummyDriveCommand(DrivetrainBase drivetrainBase){
        this.drivetrainBase = drivetrainBase;

        addRequirements(drivetrainBase);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

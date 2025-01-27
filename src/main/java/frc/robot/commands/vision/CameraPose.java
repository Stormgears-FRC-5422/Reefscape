package frc.robot.commands.vision;

import frc.robot.subsystems.VisionSubsystem;
import frc.utils.StormCommand;

import java.util.Arrays;

public class CameraPose extends StormCommand {
    VisionSubsystem visionSubsystem;
    public CameraPose(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        console("CameraPose started");
    }


    @Override
    public void execute() {
        console(Arrays.toString(visionSubsystem.getCameraPose_TargetSpace()));
        console("CameraPose execute started");
    }

}

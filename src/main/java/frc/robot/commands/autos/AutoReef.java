package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;

public class AutoReef extends StormCommand {
    DrivetrainBase drivetrainBase;
    VisionSubsystem visionSubsystem;
    ReefscapeJoystick joystick;
    int tagID = -1;
    FieldConstants.Side side;

    public AutoReef(DrivetrainBase drivetrainBase,
                    VisionSubsystem visionSubsystem,
                    ReefscapeJoystick joystick,
                    FieldConstants.Side side) {
        this.drivetrainBase = drivetrainBase;
        this.visionSubsystem = visionSubsystem;
        this.joystick = joystick;
        this.side = side;

        if (visionSubsystem.getLatestFiducialsTarget().isPresent()) {
            tagID = (int) visionSubsystem.getLatestFiducialsTarget().get().fiducialID;
        }

        addRequirements(drivetrainBase, visionSubsystem);
    }

    @Override
    public void initialize() {
        if (tagID != -1) {
            Pose2d targetPose = FieldConstants.getReefTargetPose(side, tagID);
            AutoAlign autoAlign = new AutoAlign(drivetrainBase, targetPose, joystick);
            autoAlign.schedule();
        }
    }
}

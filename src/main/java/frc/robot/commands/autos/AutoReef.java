package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;
import frc.utils.vision.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

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


        addRequirements(drivetrainBase, visionSubsystem);
    }

    @Override
    public void initialize() {
        if (LimelightHelpers.getTV(Constants.Vision.limelightID)) {
            tagID = LimelightHelpers.getRawFiducials(Constants.Vision.limelightID)[0].id;
        }
        if (tagID != -1) {
            System.out.println("AutoReef: April Tag Seen!");
            Pose2d targetPose = FieldConstants.getReefTargetPose(side, tagID);
            Logger.recordOutput("Target Pose", targetPose);

            AutoAlign autoAlign = new AutoAlign(drivetrainBase, targetPose, joystick);
            autoAlign.schedule();
        } else {
            System.out.println("No tag seen :(");
        }
    }

//    @Override
//    public boolean isFinished() {
//        return true;
//    }
}

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class AutoAlgaeReef extends AutoAlign {
    DrivetrainBase drivetrainBase;
    VisionSubsystem visionSubsystem;
    ReefscapeJoystick joystick;
    int targetTagID = -1;

    public AutoAlgaeReef(DrivetrainBase drivetrainBase,
                    VisionSubsystem visionSubsystem,
                    ReefscapeJoystick joystick,
                    int targetTagID) {

        super(drivetrainBase, joystick);
        this.drivetrainBase = drivetrainBase;
        this.visionSubsystem = visionSubsystem;
        this.joystick = joystick;
        this.targetTagID = targetTagID;

        if (visionSubsystem != null) {
            super.addRequirements(drivetrainBase, visionSubsystem);
        } else {
            super.addRequirements(drivetrainBase);
        }
    }

    public AutoAlgaeReef(DrivetrainBase drivetrainBase,
                    VisionSubsystem visionSubsystem,
                    ReefscapeJoystick joystick) {
        this(drivetrainBase, visionSubsystem, joystick, -1);
    }

    @Override
    public void initialize() {
        setTargetPose(findTargetPose());
        if (targetPose != null) {
            super.initialize();
        }
    }

    @Override
    public void execute() {
        if (targetPose != null) {
            super.execute();
        }
    }

    public Pose2d findTargetPose() {
        Pose2d targetPose = null;

        if (visionSubsystem != null) {
            if (visionSubsystem.seesTag() && targetTagID == -1) {
                targetTagID = visionSubsystem.getBestTag();
            }
        } else {
            if (Constants.Vision.simTag > 0) {
                targetTagID = Constants.Vision.simTag;
                console("Faking April Tag id:" + targetTagID);
            } else {
                return null;
            }
        }

        if (targetTagID != -1) {
            console("April Tag Seen! id:" + targetTagID);
            targetPose = FieldConstants.getAlgaeReefTargetPose(targetTagID);
//            Logger.recordOutput("Target Pose", targetPose);
            if (targetPose == null) {
                console("Didn't detect any reef AprilTag :(");
            }
        } else {
            console("No April tag detected :(");
        }
        targetTagID = -1;

        return targetPose;
    }

}

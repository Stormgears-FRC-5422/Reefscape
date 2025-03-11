package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class AutoReef extends AutoAlign {
    DrivetrainBase drivetrainBase;
    VisionSubsystem visionSubsystem;
    ReefscapeJoystick joystick;
    int tagID = -1;
    Supplier<FieldConstants.Side> sideSupplier;

    public AutoReef(DrivetrainBase drivetrainBase,
                    VisionSubsystem visionSubsystem,
                    ReefscapeJoystick joystick,
                    Supplier<FieldConstants.Side> side) {

        super(drivetrainBase, joystick);
        this.drivetrainBase = drivetrainBase;
        this.visionSubsystem = visionSubsystem;
        this.joystick = joystick;
        this.sideSupplier = side;

        if (visionSubsystem != null) {
            super.addRequirements(drivetrainBase, visionSubsystem);
        } else {
            super.addRequirements(drivetrainBase);
        }
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
        FieldConstants.Side side;
        side = sideSupplier.get();
        Pose2d targetPose = null;

        if (side == null) {
            console("side null?");
        }

        if (visionSubsystem != null) {
            if (visionSubsystem.seesTag()) {
                tagID = visionSubsystem.getBestTag();
            }
        } else {
            if (Constants.Vision.simTag > 0) {
                tagID = Constants.Vision.simTag;
                console("Faking April Tag id:" + tagID);
            } else {
                return null;
            }
        }

        if (tagID != -1) {
            console("April Tag Seen! id:" + tagID);
            targetPose = FieldConstants.getReefTargetPose(side, tagID);
            Logger.recordOutput("Target Pose", targetPose);
            if (targetPose == null) {
                console("Didn't detect any reef AprilTag :(");
            }
        } else {
            System.out.println("No April tag detected :(");
        }

        return targetPose;
    }

}

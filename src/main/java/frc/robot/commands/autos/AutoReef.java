package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class AutoReef extends AutoAlign {
    DrivetrainBase drivetrainBase;
    VisionSubsystem visionSubsystem;
    ReefscapeJoystick joystick;
    int targetTagID = -1;
    Supplier<FieldConstants.Side> sideSupplier;

    public AutoReef(DrivetrainBase drivetrainBase,
                    VisionSubsystem visionSubsystem,
                    ReefscapeJoystick joystick,
                    Supplier<FieldConstants.Side> side,
                    int targetTagID) {

        super(drivetrainBase, joystick);
        this.drivetrainBase = drivetrainBase;
        this.visionSubsystem = visionSubsystem;
        this.joystick = joystick;
        this.sideSupplier = side;
        this.targetTagID = targetTagID;

        if (visionSubsystem != null) {
            super.addRequirements(drivetrainBase, visionSubsystem);
        } else {
            super.addRequirements(drivetrainBase);
        }
    }

    public AutoReef(DrivetrainBase drivetrainBase,
                    VisionSubsystem visionSubsystem,
                    ReefscapeJoystick joystick,
                    Supplier<FieldConstants.Side> side) {
        this(drivetrainBase, visionSubsystem, joystick, side, -1);
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
            targetPose = FieldConstants.getReefTargetPose(side, targetTagID);
//            Logger.recordOutput("Target Pose", targetPose);
            if (targetPose == null) {
                console("Didn't detect any reef AprilTag :(");
            }
        } else {
            System.out.println("No April tag detected :(");
        }
        targetTagID = -1;

        return targetPose;
    }

}

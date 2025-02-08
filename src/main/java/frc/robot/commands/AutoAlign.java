package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;

import java.util.function.Supplier;

public class AutoAlign extends StormCommand {

    private final Supplier<Pose2d> targetPose;
    private DrivetrainBase drivetrainBase;
    private final ProfiledPIDController translationPID;
    private final ProfiledPIDController thetaController;


    /**
     * This command creates and follows a path.
     * Nothing else is needed to do but instantiate class
     */
    public AutoAlign(DrivetrainBase drivetrainBase, Supplier<Pose2d> targetPose) {
        this.drivetrainBase = drivetrainBase;
        this.targetPose = targetPose;
        translationPID = new ProfiledPIDController(1.5, 0.0, 0.1,
            new TrapezoidProfile.Constraints(3.0, 3.0));

        thetaController = new ProfiledPIDController(1.5, 0.0, 0.1,
            new TrapezoidProfile.Constraints(3.0, 3.0));

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrainBase);
    }


    @Override
    public void execute() {
        Pose2d currentPose = drivetrainBase.getPose();

        double distance =
            currentPose.getTranslation().getDistance(targetPose.get().getTranslation());


    }
}

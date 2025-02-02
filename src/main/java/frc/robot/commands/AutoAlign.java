package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;

public class AutoAlign extends StormCommand {

    private Pose2d targetPose = new Pose2d();
    private DrivetrainBase drivetrainBase;
    private final PIDController translationPID = new PIDController(1.0, 0, 0);
    private final ProfiledPIDController rotationPID = new ProfiledPIDController(
        1.5, 0.0, 0.1,
        new TrapezoidProfile.Constraints(3.0, 3.0)
    );



    /**
     * This command creates and follows a path.
     * Nothing else is needed to do but instantiate class
     */
    public AutoAlign(DrivetrainBase drivetrainBase, Pose2d targetPose) {
        this.drivetrainBase = drivetrainBase;
        this.targetPose = targetPose;


        addRequirements(drivetrainBase);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrainBase.getPose();

        Translation2d error = targetPose.getTranslation().minus(currentPose.getTranslation());
        double distance = error.getNorm();

        double translationSpeed = translationPID.calculate(distance, 0);

        Translation2d direction = error.div(distance).times(translationSpeed);

        double angleError = targetPose.getRotation().minus(currentPose.getRotation()).getRadians();
        double rotationSpeed = rotationPID.calculate(angleError, 0);

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            direction.getX(), direction.getY(), rotationSpeed, currentPose.getRotation()
        );

        drivetrainBase.drive(speeds, true);

    }
}

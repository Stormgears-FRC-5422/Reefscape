package frc.robot.commands.autos;

import choreo.Choreo;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.Optional;

public class AutoCommandFactory extends Command {
    private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("test");
    static DrivetrainBase drivetrainBase;
    private static final PIDController xController = new PIDController(0, 0.0, 0.0);
    private static final PIDController yController = new PIDController(0, 0.0, 0.0);
    private static final PIDController headingController = new PIDController(0, 0.0, 0.0);
    private final Timer timer = new Timer();


    public AutoCommandFactory(DrivetrainBase drivetrainBase) {
        this.drivetrainBase = drivetrainBase;
    }

    public static void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = drivetrainBase.getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        drivetrainBase.drive(speeds, true);
    }

    @Override
    public void execute() {
        Commands.runOnce(timer::restart);
        Optional<SwerveSample> sample = trajectory.get().sampleAt(timer.get(), false);

        sample.ifPresent(AutoCommandFactory::followTrajectory);

    }
}

package frc.robot.commands.autos;

import choreo.Choreo;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class AutoCommandFactory extends Command {
    private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("test3");
    static DrivetrainBase drivetrainBase;
    private static final PIDController xController = new PIDController(2.6, 0.0, 0.1);
    private static final PIDController yController = new PIDController(2.6, 0.0, 0.1);
    private static final PIDController headingController = new PIDController(3.1, 0.0, 0.1);
    private Timer timer;
    private int count = 0;


    public AutoCommandFactory(DrivetrainBase drivetrainBase) {
        this.drivetrainBase = drivetrainBase;
        timer = new Timer();
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrainBase);
    }

    public void followTrajectory(SwerveSample sample) {
        if (trajectory.isPresent()) {
            trajectory.get().getInitialPose(RobotState.getInstance().isAllianceRed());
        }
        // Get the current pose of the robot
        Pose2d pose = drivetrainBase.getPose();
//        Logger.recordOutput("Auto/pose trj x", sample.x);
        Logger.recordOutput("Auto/pose chass speeds", sample.getChassisSpeeds());

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );
        Logger.recordOutput("Auto/header", sample.heading);
        Logger.recordOutput("Auto/omega", sample.omega);
        Logger.recordOutput("Auto/speed", speeds);
        drivetrainBase.drive(speeds, true);
    }

    @Override
    public void initialize() {
        if (trajectory.isPresent()) {
            drivetrainBase.declarePoseIsNow(trajectory.get().getInitialPose(false).get());
        }
        count = 0;
        if (trajectory.isPresent() && trajectory.get().getInitialPose(RobotState.getInstance().isAllianceRed()).isPresent()) {
            drivetrainBase.declarePoseIsNow(trajectory.get().getInitialPose(RobotState.getInstance().isAllianceRed()).get());
        }
    }

    @Override
    public void execute() {
        if (count == 0) {
            timer.restart();
            count++;
        }

        double time = timer.get();
        Optional<SwerveSample> sample = trajectory.get().sampleAt(time, RobotState.getInstance().isAllianceRed());
//        Optional<SwerveSample> sample = trajectory.get().sampleAt(0.27225, false);
        Logger.recordOutput("Auto/timer timer", time);

        if (sample.isPresent()) {
            followTrajectory(sample.get());
        } else {
            System.out.println("no");
        }
        Logger.recordOutput("Auto/pose trj y", trajectory.get().sampleAt(time, RobotState.getInstance().isAllianceRed()).get().getPose().getY());


    }

    @Override
    public void end(boolean interrupted) {
    }
}

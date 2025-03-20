// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.AKdrive.AKDriveInternal;
import frc.robot.subsystems.drive.AKdrive.SimpleTelemetry;
import frc.utils.vision.LimelightHelpers;
import org.littletonrobotics.junction.AutoLogOutput;
import frc.robot.subsystems.drive.ctrGenerated.TunerConstantsWrapper;

public class AKDriveTrain extends DrivetrainBase {
    private AKDriveInternal driveInternal;

    private SwerveDrivePoseEstimator poseEstimator;
    private final StructPublisher<Pose2d> publisher;

    private final SimpleTelemetry robotTelemetry = new SimpleTelemetry("robotPose");
    private final SimpleTelemetry visionTelemetry = new SimpleTelemetry("visionPose");

    public AKDriveTrain(Class<?> tunerConstantsClass) {
        TunerConstantsWrapper tunerConstants = new TunerConstantsWrapper(tunerConstantsClass);
        AKDriveInternal.useTunerConstants(tunerConstants);
        driveInternal = new AKDriveInternal();

        poseEstimator = driveInternal.getPoseEstimator();
        double maxSpeed = driveInternal.getMaxLinearSpeedMetersPerSec();
        double maxAngularRate = driveInternal.getMaxAngularSpeedRadPerSec();

        setMaxVelocities(maxSpeed, maxAngularRate);

        publisher = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose2d.struct).publish();
    }

    /**
     * Returns the current odometry pose.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    @Override
    public Pose2d getPose() {
        return driveInternal.getPose();
    }

    /**
     * Returns the current odometry rotation.
     */
    @Override
    public Rotation2d getRotation() {
        return driveInternal.getRotation();
    }

    @Override
    public void resetOrientation() {
        // Note that this behavior defaults to blue if alliance is missing.
        Rotation2d newRotation = m_state.isAllianceRed()
            ? new Rotation2d(-1, 0)
            : new Rotation2d(1, 0);

        driveInternal.resetRotation(newRotation);
//        System.out.println("wow");
    }

    /**
     * Resets the current odometry pose.
     */
    @Override
    public void declarePoseIsNow(Pose2d pose) {
        driveInternal.resetPosition(pose);
    }

    /*
    Set the gyro to the MegaTag2
     */
    @Override
    public void setGyroMT2() {
        LimelightHelpers.SetRobotOrientation("", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Adds a new timestamped vision measurement.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters,
                                     double timestampSeconds,
                                     Matrix<N3, N1> visionMeasurementStdDevs) {
        driveInternal.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        super.periodic();

        driveInternal.setChassisSpeeds(m_chassisSpeeds);
        driveInternal.runVelocity(m_chassisSpeeds);
        driveInternal.periodic();

        Pose2d currentPose = getPose();
        m_state.setPose(currentPose);

        RobotState.VisionMeasurement m = m_state.getVisionMeasurment();
        Pose2d visionPose = new Pose2d();
        if (m != null) {
            visionPose = m.visionRobotPoseMeters();
            addVisionMeasurement(visionPose, m.timestampSeconds(), m.visionMeasurementStdDevs());
        }

        publisher.set(getPose());
        // TODO - add a config to add these during simulation
        robotTelemetry.telemeterize(currentPose);
        visionTelemetry.telemeterize(visionPose);
    }

    @Override
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return driveInternal.sysIdQuasistatic(direction);
    }

    @Override
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return driveInternal.sysIdDynamic(direction);
    }


    @Override
    public double getFFCharacterizationVelocity() {
        return super.getFFCharacterizationVelocity();
    }

    @Override
    public void goToZero() {
        driveInternal.goToZero();
    }

    @Override
    public void goToNinety() {
        driveInternal.goToNinety();
    }

    @Override
    public void runCharacterization(double output) {
        super.runCharacterization(output);
    }

    @Override
    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }
}

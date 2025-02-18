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
import frc.robot.subsystems.drive.AKdrive.AKDriveInternal;
import frc.utils.vision.LimelightHelpers;
import org.littletonrobotics.junction.AutoLogOutput;
import frc.robot.subsystems.drive.ctrGenerated.TunerConstantsWrapper;

public class AKDriveTrain extends DrivetrainBase {
    private AKDriveInternal driveInternal;

    private SwerveDrivePoseEstimator poseEstimator;

    public AKDriveTrain(Class<?> tunerConstantsClass) {
        TunerConstantsWrapper tunerConstants = new TunerConstantsWrapper(tunerConstantsClass);
        AKDriveInternal.useTunerConstants(tunerConstants);
        driveInternal = new AKDriveInternal();

        poseEstimator = driveInternal.getPoseEstimator();
        double maxSpeed = driveInternal.getMaxLinearSpeedMetersPerSec();
        double maxAngularRate = driveInternal.getMaxAngularSpeedRadPerSec();

        setMaxVelocities(maxSpeed, maxAngularRate);
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
    public Rotation2d getRotation() { return driveInternal.getRotation(); }

    @Override
    public void resetOrientation() {
        // Note that this behavior defaults to blue if alliance is missing.
        Rotation2d newRotation = m_state.isAllianceRed()
            ? new Rotation2d(-1, 0)
            : new Rotation2d(1, 0);

        driveInternal.resetRotation(newRotation);
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

        if (m_state.getVisionMeasurments() != null) {
            double tRadians = m_state.getVisionMeasurments().visionRobotPoseMeters().getRotation().getRadians();
            Pose2d tempPose = new Pose2d(m_state.getVisionMeasurments().visionRobotPoseMeters().getTranslation(),
                new Rotation2d(tRadians));

//          console(m_state.getVisionMeasurements().visionRobotPoseMeters().getRotation(),25);
            addVisionMeasurement(tempPose,
                m_state.getVisionMeasurments().timestampSeconds(),
                m_state.getVisionMeasurments().visionMeasurementStdDevs());
        }

        driveInternal.setChassisSpeeds(m_chassisSpeeds);
        driveInternal.runVelocity(m_chassisSpeeds);
        driveInternal.periodic();

        m_state.setPose(getPose());
    }
}

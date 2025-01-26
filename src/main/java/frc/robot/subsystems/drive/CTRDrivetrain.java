package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.ctrGenerated.CTRDriveInternal;
import frc.robot.subsystems.drive.ctrGenerated.Telemetry;
import org.littletonrobotics.junction.AutoLogOutput;

import java.lang.reflect.Field;
import java.lang.reflect.Method;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class CTRDrivetrain <T> extends DrivetrainBase {
    public final CTRDriveInternal drivetrain;
    private final SwerveRequest.FieldCentric driveFieldCentric;
    private final SwerveRequest.RobotCentric driveRobotCentric;

    RobotState robotState;
    SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    DoubleArraySubscriber poseSub;
    int count = 0;
    Telemetry logger;

    public CTRDrivetrain(T tunerConstants) {
        super();

        double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
        double MaxSpeed = getMaxSpeed(tunerConstants);
        drivetrain = getInternalDriveTrain(tunerConstants);

        logger = new Telemetry(MaxSpeed);

        driveFieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        setMaxVelocities(MaxSpeed, MaxAngularRate);

        // This class will flip for alliance as necessary, rather than relying on base class
        setDriveFlip(true);
        // This class will handle field-relative converstions,  rather than relying on base class
        setFieldRelativeOn(false);

//        if (Utils.isSimulation()) {
//            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
//        }
        drivetrain.resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

// Reflection functions
    double getMaxSpeed(T tunerConstants) {
        double maxSpeed = 0;
        try {
            Field field = ((Class<?>) tunerConstants).getField("kSpeedAt12Volts");
            Object kSpeedAt12Volts = field.get(null); // Static field, so null for instance
            return ((LinearVelocity) kSpeedAt12Volts).in(MetersPerSecond);
        } catch (Exception e) {
            throw new RuntimeException("Failed to access kSpeedAt12Volts: " + e.getMessage(), e);
        }
    }

    CTRDriveInternal getInternalDriveTrain(T tunerConstants) {
        try {
            // tunerConstants is a Class<?> reference
            Method method = ((Class<?>) tunerConstants).getMethod("createDrivetrain");
            return (CTRDriveInternal) method.invoke(null); // Static method, so null for instance
        } catch (Exception e) {
            throw new RuntimeException("Failed to create drivetrain: " + e.getMessage(), e);
        }
    }

// end of reflection functions

    @Override
    public void resetOrientation() {
        // Note that this behavior defaults to blue if alliance is missing.
        Rotation2d newRotation = m_state.isAllianceRed() ? new Rotation2d(-1, 0) : new Rotation2d(1, 0);
        console("Reset orientation at degrees: " + newRotation.getDegrees());
        drivetrain.getPigeon2().setYaw(newRotation.getDegrees());
    }

    public void declarePoseIsNow(Pose2d newPose) {
        // These *MUST* be done in this order or odometry will be wrong.
        drivetrain.getPigeon2().setYaw(newPose.getRotation().getDegrees());
        resetPose(newPose);
        m_state.setPose(newPose);
    }

    public void resetPose(Pose2d pose) {
//        drivetrain.getPoseEstimator().resetPosition(drivetrain.getPigeon2().getRotation2d(), drivetrain.getModulePositions(), pose);
    }

    @AutoLogOutput
    public Pose2d getPose() {
        return new Pose2d();
//        return new Pose2d(poseSub.get()[0], poseSub.get()[1], new Rotation2d(Math.toRadians(poseSub.get()[2])));
    }

    @Override
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, double speedScale) {
        super.drive(speeds, fieldRelative, speedScale);
        console("Drive/DesiredSpeeds" + speeds,100);
    }

    @Override
    public void periodic() {
        super.periodic();
        drivetrain.periodic();

        if (m_state.getPeriod() == RobotState.StatePeriod.AUTONOMOUS) {
            console("in CTRDrive chassisSpeeds: " + m_chassisSpeeds, 100);
        }

        if (m_fieldRelative) {
            drivetrain.setControl(driveFieldCentric.withVelocityX(m_chassisSpeeds.vxMetersPerSecond).withVelocityY(m_chassisSpeeds.vyMetersPerSecond).withRotationalRate(m_chassisSpeeds.omegaRadiansPerSecond));
        } else {
            drivetrain.setControl(driveRobotCentric.withVelocityX(m_chassisSpeeds.vxMetersPerSecond).withVelocityY(m_chassisSpeeds.vyMetersPerSecond).withRotationalRate(m_chassisSpeeds.omegaRadiansPerSecond));
        }
    }
}

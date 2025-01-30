package frc.robot.subsystems.drive;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drive;
import frc.robot.RobotState;
import frc.robot.ShuffleboardConstants;
import frc.robot.subsystems.drive.ctrGenerated.ReefscapeTunerConstants;
import frc.utils.StormSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;

public abstract class DrivetrainBase extends StormSubsystem {

    public double m_maxVelocityMetersPerSecond = 1;
    public double m_maxAngularVelocityRadiansPerSecond = 1;
    protected double m_driveSpeedScale = 0;
    private final SlewRateLimiter speedScaleLimiter = new SlewRateLimiter(0.25);
    protected final ShuffleboardTab tab;
    protected

    final RobotState m_state;
    protected boolean m_fieldRelative = false;
    public static boolean driveFlip = true;
    public static boolean fieldRelativeOn = true;

    @AutoLogOutput
    protected ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    //    protected final ShuffleboardTab tab;
    //    protected final ShuffleboardTab tab;
    public DrivetrainBase() {
        setDriveSpeedScale(Drive.driveSpeedScale);
        tab = ShuffleboardConstants.getInstance().drivetrainTab;
        m_state = RobotState.getInstance();
        setDriveFlip(false);
        setFieldRelativeOn(false);
    }



    protected void setMaxVelocities(double maxVelocityMetersPerSecond, double maxAngularVelocityRadiansPerSecond) {
        m_maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
        m_maxAngularVelocityRadiansPerSecond = maxAngularVelocityRadiansPerSecond;
        System.out.println("MaxDRIVEVal: " + m_maxVelocityMetersPerSecond);
        System.out.println("MaxANgleVal: " + m_maxAngularVelocityRadiansPerSecond);
    }

    // Be careful scaling ChassisSpeeds. Need to scale X and Y the same or your robot will move in the wrong direction!
    public ChassisSpeeds scaleChassisSpeeds(ChassisSpeeds speeds, double scale) {
        return new ChassisSpeeds(scale * speeds.vxMetersPerSecond,
            scale * speeds.vyMetersPerSecond,
            scale * speeds.omegaRadiansPerSecond);
    }

    /**
     * Command the robot to drive.
     * This method expects real speeds in meters/second.
     * Speed may be limited by speedScale and / or slew rate limiter
     *
     * @param speeds        Chassis speedsSwerveDriveConfiguration for the swerve, esp from joystick.
     * @param fieldRelative True for field relative driving
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(speeds, fieldRelative, m_driveSpeedScale);
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative, double speedScale) {
        m_fieldRelative = fieldRelative;

        if (fieldRelativeOn && fieldRelative) {
            Rotation2d rotation = m_state.getHeading();
            m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation);
        } else {
            m_chassisSpeeds = speeds;
        }

        // TODO - work in the slew rate limiter. Apply before scale to preserve motion details
        m_chassisSpeeds = scaleChassisSpeeds(m_chassisSpeeds, speedScale);
    }

    /**
     * Command the robot to drive, especially from Joystick
     * This method expects units from -1 to 1, and then scales them to the max speeds
     * You should call setMaxVelocities() before calling this method
     *
     * @param speeds        Chassis speeds, especially from joystick.
     * @param fieldRelative True for field relative driving
     */
    public void percentOutputDrive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(new ChassisSpeeds(speeds.vxMetersPerSecond * m_maxVelocityMetersPerSecond,
                speeds.vyMetersPerSecond * m_maxVelocityMetersPerSecond,
                speeds.omegaRadiansPerSecond * m_maxAngularVelocityRadiansPerSecond),
            fieldRelative);
    }

    public void setDriveSpeedScale(double scale) {
        m_driveSpeedScale = MathUtil.clamp(scale, 0, Drive.driveSpeedScale);
    }

    public void stopDrive() {
        drive(new ChassisSpeeds(0, 0, 0), false);
    }

    public ChassisSpeeds getCurrentChassisSpeeds() {
        return m_chassisSpeeds;
    }

    public Pose2d getPose() {
        return new Pose2d();
    }

    public void runCharacterization(double output){

    }

    /**
     * Returns the average velocity of the modules in rotations/sec (Phoenix native units).
     */
    public double getFFCharacterizationVelocity() {
      return 0.0;
    }


        // Teach the drive that the current orientation is facing the opposite end of the field
    // this function is ideally alliance aware, and manages the pose and gyro as needed
    public void resetOrientation() {
    }

    public void declarePoseIsNow(Pose2d pose) {
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters,
                                     double timestampSeconds,
                                     Matrix<N3, N1> visionMeasurementStdDevs) {

    }

    public Rotation2d getRotation() {
        return new Rotation2d();
    }

    public Command getQuasForwardCommand() {
        return new Command() {
        };
    }

    public Command getQuasBackwardCommand() {
        return new Command() {
        };
    }

    public Command getDynamicForwardCommand() {
        return new Command() {
        };
    }

    public Command getDynamicBackwardCommand() {
        return new Command() {
        };
    }

    public void zeroWheels() {

    }

    public void setHeadingCorrectionTrue() {
    }

    public Command getSysIdCommand() {
        return new Command() {
        };

    }



    public SwerveDriveKinematics getKinematics() {
        return new SwerveDriveKinematics();
    }


    public void setSwerveModulesStates(SwerveModuleState[] swerveModulesStates) {

    }

    protected void setDriveFlip(boolean flip) {
        driveFlip = flip;
    }

    protected void setFieldRelativeOn(boolean flip) {
        fieldRelativeOn = flip;
    }
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[0];
        return values;

    }
        public static final double DRIVE_BASE_RADIUS =
        Math.max(
            Math.max(
                Math.hypot(ReefscapeTunerConstants.FrontLeft.LocationX, ReefscapeTunerConstants.FrontLeft.LocationY),
                Math.hypot(ReefscapeTunerConstants.FrontRight.LocationX, ReefscapeTunerConstants.FrontRight.LocationY)),
            Math.max(
                Math.hypot(ReefscapeTunerConstants.BackLeft.LocationX, ReefscapeTunerConstants.BackLeft.LocationY),
                Math.hypot(ReefscapeTunerConstants.BackRight.LocationX, ReefscapeTunerConstants.BackRight.LocationY)));
}


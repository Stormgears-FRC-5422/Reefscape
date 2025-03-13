package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Drive;
import frc.robot.RobotState;
import frc.robot.ShuffleboardConstants;
import frc.robot.subsystems.drive.ctrGenerated.ReefscapeTunerConstants;
import frc.utils.StormSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public abstract class DrivetrainBase extends StormSubsystem {
    public static final double DRIVE_BASE_RADIUS =
        Math.max(
            Math.max(
                Math.hypot(ReefscapeTunerConstants.FrontLeft.LocationX, ReefscapeTunerConstants.FrontLeft.LocationY),
                Math.hypot(ReefscapeTunerConstants.FrontRight.LocationX, ReefscapeTunerConstants.FrontRight.LocationY)),
            Math.max(
                Math.hypot(ReefscapeTunerConstants.BackLeft.LocationX, ReefscapeTunerConstants.BackLeft.LocationY),
                Math.hypot(ReefscapeTunerConstants.BackRight.LocationX, ReefscapeTunerConstants.BackRight.LocationY)));

    public static boolean driveFlip = true;
    public static boolean fieldRelativeOn = true;
    protected final ShuffleboardTab tab;
    protected

    final RobotState m_state;
    private final SlewRateLimiter speedScaleLimiter = new SlewRateLimiter(0.25);
    public double m_maxVelocityMetersPerSecond = 1;
    public double m_maxAngularVelocityRadiansPerSecond = 1;
    protected double m_driveSpeedScale = 0;
    protected boolean m_fieldRelative = false;
    private static final PIDController xController = new PIDController(2.1, 0.0, 0.1);
    private static final PIDController yController = new PIDController(2.6, 0.0, 0.1);
    private static final PIDController headingController = new PIDController(3.1, 0.0, 0.1);
    @AutoLogOutput
    protected ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DrivetrainBase() {
        setDriveSpeedScale(Drive.driveSpeedScale);
        tab = ShuffleboardConstants.getInstance().drivetrainTab;
        m_state = RobotState.getInstance();
        setDriveFlip(false);
        setFieldRelativeOn(false);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    protected void setDriveFlip(boolean flip) {
        driveFlip = flip;
    }

    protected void setFieldRelativeOn(boolean flip) {
        fieldRelativeOn = flip;
    }

    protected void setMaxVelocities(double maxVelocityMetersPerSecond, double maxAngularVelocityRadiansPerSecond) {
        m_maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
        m_maxAngularVelocityRadiansPerSecond = maxAngularVelocityRadiansPerSecond;
        System.out.println("MaxDriveVelocity: " + m_maxVelocityMetersPerSecond);
        System.out.println("MaxAngularVelocity: " + m_maxAngularVelocityRadiansPerSecond);
    }

    public void setDriveSpeedScale(double scale) {
        m_driveSpeedScale = MathUtil.clamp(scale, 0, Drive.driveSpeedScale);
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
        setFieldRelativeOn(true);

        if (fieldRelativeOn && fieldRelative) {
            Rotation2d rotation = getRotation();
            m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation);
        } else {
            m_chassisSpeeds = speeds;
        }

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

    @AutoLogOutput
    public ChassisSpeeds getCurrentChassisSpeeds() {
        return m_chassisSpeeds;
    }

    public Pose2d getPose() {
        return new Pose2d();
    }

    public Rotation2d getRotation() {
        return new Rotation2d();
    }

    // Teach the drive that the current orientation is facing the opposite end of the field
    // this function is ideally alliance aware, and manages the pose and gyro as needed
    public void resetOrientation() {
    }

    public void declarePoseIsNow(Pose2d pose) {
    }

    public void setGyroMT2() {
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters,
                                     double timestampSeconds,
                                     Matrix<N3, N1> visionMeasurementStdDevs) {

    }

    public void runCharacterization(double output) {
    }

    /**
     * Returns the average velocity of the modules in rotations/sec (Phoenix native units).
     */
    public double getFFCharacterizationVelocity() {
        return 0.0;
    }

    public void goToZero(){

    }

    public void goToNinety(){

    }

    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[0];
        return values;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return new Command() {
        };
    }

    /**
     * Returns a command to run a dynamic test in the specified direction.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return new Command() {
        };
    }

    public SwerveDrivePoseEstimator getPoseEstimator(){
        return null;
    }

    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = getPose();

        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        Logger.recordOutput("Auto/setpoint chassis speed", speeds);
        Logger.recordOutput("Auto/trajectory pose", sample.getPose());
        drive(speeds, true);
    }
}


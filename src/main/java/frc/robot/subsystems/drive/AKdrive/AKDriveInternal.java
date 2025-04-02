package frc.robot.subsystems.drive.AKdrive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import frc.robot.Constants;
import frc.robot.RobotState;
import frc.utils.swerve.ModuleLimits;
import frc.utils.swerve.SwerveSetpoint;
import frc.utils.swerve.SwerveSetpointGenerator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.drive.ctrGenerated.TunerConstantsWrapper;

public class AKDriveInternal implements Subsystem {
    private static TunerConstantsWrapper TunerConstants;

    // TunerConstants doesn't include these constants, so they are declared locally
    static double ODOMETRY_FREQUENCY;
    static public double DRIVE_BASE_RADIUS;

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert =
        new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    public Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
        new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };
    private boolean correctionEnabled;
    private double lastHeadingRadians;
    private PIDController headingController = new PIDController(3.6, 0.0, 0.1);

    private SwerveDrivePoseEstimator poseEstimator =
        new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    private final SwerveSetpointGenerator setpointGenerator;
    private final ModuleLimits moduleLimits;
    private SwerveSetpoint lastSwerveSetpoint;


    ChassisSpeeds m_chassisSpeeds;

    // This function MUST be called before the constructor. Ideally we'd make a factory
    // to ensure this. We should unify with CTRDrive at that point.
    static public void useTunerConstants(TunerConstantsWrapper tunerConstants) {
        TunerConstants = tunerConstants;
    }

    public AKDriveInternal() {
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        // These geometry variables must be assigned first
        ODOMETRY_FREQUENCY =
            new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;

        DRIVE_BASE_RADIUS =
            Math.max(
                Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
                Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));


        this.gyroIO = new GyroIOPigeon2();
        if (RobotState.getInstance().getSimMode().equals(RobotState.StateSimMode.AKIT_SIM)) {
            modules[0] = new Module(new ModuleIOSim(TunerConstants.FrontLeft),
                0, TunerConstants.FrontLeft);
            modules[1] = new Module(new ModuleIOSim(TunerConstants.FrontRight),
                1, TunerConstants.FrontRight);
            modules[2] = new Module(new ModuleIOSim(TunerConstants.BackLeft),
                2, TunerConstants.BackLeft);
            modules[3] = new Module(new ModuleIOSim(TunerConstants.BackRight),
                3, TunerConstants.BackRight);
        } else {
            modules[0] = new Module(new ModuleIOTalonFX(TunerConstants.FrontLeft),
                0, TunerConstants.FrontLeft);
            modules[1] = new Module(new ModuleIOTalonFX(TunerConstants.FrontRight),
                1, TunerConstants.FrontRight);
            modules[2] = new Module(new ModuleIOTalonFX(TunerConstants.BackLeft),
                2, TunerConstants.BackLeft);
            modules[3] = new Module(new ModuleIOTalonFX(TunerConstants.BackRight),
                3, TunerConstants.BackRight);
        }

        // Usage reporting for swerve template
        HAL.report(FRCNetComm.tResourceType.kResourceType_RobotDrive, FRCNetComm.tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();


        // Configure SysId
//        sysId =
//            new SysIdRoutine(
//                new SysIdRoutine.Config(
//                    null,
//                    null,
//                    null,
//                    (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
//                new SysIdRoutine.Mechanism(
//                    (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
        // Configure SysId
        sysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> runTurnCharacterization(voltage.in(Volts)), null, this));

        setpointGenerator =
            SwerveSetpointGenerator.builder()
                .kinematics(kinematics)
                .moduleLocations(getModuleTranslations())
                .build();

        moduleLimits = new ModuleLimits(getMaxLinearSpeedMetersPerSec(),
            Units.feetToMeters(60), getMaxAngularSpeedRadPerSec());

        lastSwerveSetpoint = new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[]{
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
            });
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public void resetPosition(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    public void resetRotation(Rotation2d rotation) {
        poseEstimator.resetPose(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(),
            rotation));

    }

    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);

        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[]{});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[]{});
        }

        // Update odometry
        double[] sampleTimestamps =
            modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] =
                    new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                            - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected
                && RobotState.getInstance().getSimMode().equals(RobotState.StateSimMode.REAL)) {
//                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
//                 Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }


            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && CTRConstants.currentMode != CTRConstants.Mode.SIM);

        Logger.recordOutput("measured angle 0", MathUtil.angleModulus(getModuleStates()[0].angle.getRadians()));
//        System.out.println("measure angle 1 " +  MathUtil.angleModulus(getModuleStates()[1].angle.getRadians()));
        Logger.recordOutput("measured angle 1", MathUtil.angleModulus(getModuleStates()[1].angle.getRadians()));
        Logger.recordOutput("measured angle 2", MathUtil.angleModulus(getModuleStates()[2].angle.getRadians()));
        Logger.recordOutput("measured angle 3", MathUtil.angleModulus(getModuleStates()[3].angle.getRadians()));
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        if (Constants.Toggles.useSetPointGenerator) {
            lastSwerveSetpoint = setpointGenerator.generateSetpoint(
                moduleLimits, lastSwerveSetpoint, discreteSpeeds, 0.02
            );
            setpointStates = lastSwerveSetpoint.moduleStates();

        } else {
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);
        }

//        var tmpStates = kinematics.toSwerveModuleStates(speeds);
//        SwerveDriveKinematics.desaturateWheelSpeeds(tmpStates, TunerConstants.kSpeedAt12Volts);
//
//        var speedsConverted = kinematics.toChassisSpeeds(tmpStates);
//
//        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speedsConverted, 0.02);
//
//        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
//        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);



        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);


        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i], i + 1);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /**
     * Runs the drive in a straight line with the specified drive output.
     */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }


    public void runTurnCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runTurnCharacterization(output);
        }
    }

    public void runAngleMotionCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runAngularMotionCharacterization(output);
        }
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /**
     * Returns a command to run a quasistatic test in the specified direction.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runTurnCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(sysId.quasistatic(direction));
    }

//    /**
//     * Returns a command to run a quasistatic test in the specified direction.
//     */
//    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
//        return run(() -> runCharacterization(0.0))
//            .withTimeout(1.0)
//            .andThen(sysId.quasistatic(direction));
//    }
//
//    /**
//     * Returns a command to run a dynamic test in the specified direction.
//     */
//    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
//        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
//    }

    /**
     * Returns a command to run a dynamic test in the specified direction.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runTurnCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /**
     * Returns the measured chassis speeds of the robot.
     */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")

    protected ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Returns the position of each module in radians.
     */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /**
     * Returns the average velocity of the modules in rotations/sec (Phoenix native units).
     */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /**
     * Returns the current odometry pose.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the current odometry rotation.
     */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public void goToZero() {
        for (Module module : modules) {
            module.setAngle(new Rotation2d());
        }
    }

    public void goToNinety() {
        for (Module module : modules) {
            module.setAngle(new Rotation2d(Math.PI / 2));
        }
    }


    /**
     * Resets the current odometry pose.
     */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /**
     * Adds a new timestamped vision measurement.
     */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(
            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /**
     * Returns an array of module translations.
     */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[]{
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

}

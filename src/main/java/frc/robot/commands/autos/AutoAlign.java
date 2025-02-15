package frc.robot.commands.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotState;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;

import java.util.function.Supplier;

public class AutoAlign extends StormCommand {

    private final Pose2d targetPose;
    private Supplier<Pose2d> goalPose;
    private ReefscapeJoystick joystick;
    private RobotState robotState;
    private DrivetrainBase drivetrainBase;
    private final ProfiledPIDController translationPID;
    private final ProfiledPIDController thetaController;
    private final double maxVelocity = 1.5;
    private Translation2d driverAdjustment;
    private final double ffMinDistance = 0.2;
    private final double ffMaxDistance = 0.8;
    private Translation2d lastSetpointTranslation;
    private final double loopTimeSec = 0.02;
    private Translation2d feedForward;
    private final double linearTolerance = 0.08;
    private final double thetaTolerance = Units.degreesToRadians(2.5);
    private Timer timer;
    private boolean flag = false;


    /**
     * This command creates and follows a path.
     */
    public AutoAlign(DrivetrainBase drivetrainBase, Pose2d targetPose, ReefscapeJoystick joystick) {
        timer = new Timer();
        feedForward = new Translation2d();
        robotState = RobotState.getInstance();
        driverAdjustment = new Translation2d();
        this.joystick = joystick;

        this.drivetrainBase = drivetrainBase;
        this.targetPose = targetPose;
        translationPID = new ProfiledPIDController(3.5, 0.0, 0.1,
            new TrapezoidProfile.Constraints(1.5, 1.5));
        translationPID.setTolerance(linearTolerance);


        thetaController = new ProfiledPIDController(3.2, 0.0, 0.1,
            new TrapezoidProfile.Constraints(1.5, 1.5));
        thetaController.setTolerance(thetaTolerance);


        thetaController.enableContinuousInput(-Math.PI, Math.PI);


    }

    @Override
    public void initialize() {
        timer.restart();
        goalPose = () -> {
            double linearMagnitude = MathUtil.applyDeadband(Math.hypot(joystick.getWpiX(),
                joystick.getWpiY()), 0.1);
            Translation2d linearVelocity = new Translation2d(joystick.getWpiX(),
                joystick.getWpiY()).times(linearMagnitude);

            Pose2d finalPose = new Pose2d(targetPose.getX(), targetPose.getY(),targetPose.getRotation());
            //scale by maxVelocity and then get movement in one periodic cycle
            feedForward = linearVelocity
                .times(maxVelocity);
            driverAdjustment = feedForward.times(loopTimeSec);
            double distance = drivetrainBase.getPose().getTranslation()
                .minus(driverAdjustment)
                .getDistance(finalPose.getTranslation());


//            (distance - 0.3) makes adjustment sooner if number is smaller and later if number is bigger
//            dividing by 2.5 makes robot less sensitive to changes: increase to make less and decrease to make more
            double offsetT = MathUtil.clamp((distance - 0.3) / 2.5, 0.0, 1.0);
//            multiply by 1.75 causes correction to bigger: decrease to make corrections smaller
//            and vice versa. Decrease for less overshoot
            return finalPose.transformBy(new Transform2d(0.0, offsetT * 1.0 , new Rotation2d()));
        };
        Pose2d currentPose = drivetrainBase.getPose();
        Pose2d currentGoalPose = goalPose.get();
        Twist2d fieldVelocity = new Twist2d(drivetrainBase.getCurrentChassisSpeeds().vxMetersPerSecond,
            drivetrainBase.getCurrentChassisSpeeds().vyMetersPerSecond,
            drivetrainBase.getCurrentChassisSpeeds().omegaRadiansPerSecond);
        Rotation2d robotToGoalAngle =
            currentGoalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
//        this puts the velocity in coordinate systems from the target and ensures it is not moving away
        double linearVelocity =
            Math.min(
                0.0,
                -new Translation2d(fieldVelocity.dx, fieldVelocity.dy)
                    .rotateBy(robotToGoalAngle.unaryMinus())
                    .getX());
        translationPID.reset(
            currentPose.getTranslation().getDistance(currentGoalPose.getTranslation()), linearVelocity);
        thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.dtheta);
        lastSetpointTranslation = currentPose.getTranslation();
    }

    @Override
    public void execute() {
        Pose2d currentGoalPose = goalPose.get();
        Pose2d currentPose = drivetrainBase.getPose();

        double distance = currentPose.getTranslation().getDistance(currentGoalPose.getTranslation());

//        feedforward value based on distance away from target in the rang from min to max
//        value is a fraction based on distance between range
        double ffScaler =
            MathUtil.clamp(
                (distance - ffMinDistance) / (ffMaxDistance - ffMinDistance),
                0.0,
                1.0);

//        reset the pid controller with the controller's velocity so it continues to follow path it's on
//        and not snap to different velocity
        translationPID.reset(
            lastSetpointTranslation.getDistance(currentGoalPose.getTranslation()),
            translationPID.getSetpoint().velocity);

//        combine both feedforward and feedback to get drive velocity
        double driveVelocityScalar =
            translationPID.getSetpoint().velocity * ffScaler
                + translationPID.calculate(distance, 0.0);

//        calculate last setpoint by getting pose of target and transform by adding the next setpoint
//        we should not have to construct a new pose but leaving it bc I am not 100% sure
        lastSetpointTranslation =
            new Pose2d(
                goalPose.get().getTranslation(),
                currentPose.getTranslation().minus(currentGoalPose.getTranslation()).getAngle())
                .transformBy(new Transform2d(translationPID.getSetpoint().position, 0.0, new Rotation2d()))
                .getTranslation();


        // Calculate theta speed just like drive
        double thetaVelocity =
            thetaController.getSetpoint().velocity * ffScaler
                + thetaController.calculate(
                currentPose.getRotation().getRadians(), currentGoalPose.getRotation().getRadians());


//        we also add feedforward so the velocity of the actual robot changes not only the goal pose
//        when driver moves joystick
        Translation2d driveVelocity =
            new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(currentGoalPose.getTranslation()).getAngle())
                .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
                .getTranslation()
                .plus(feedForward);


        drivetrainBase.drive(new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity),
            true);

    }

    @Override
    public boolean isFinished() {
        return (translationPID.atGoal() && thetaController.atGoal()) ||
            (timer.get()>4);
    }


}



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
import frc.robot.subsystems.drive.AKdrive.SimpleTelemetry;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class AutoAlign extends StormCommand {

    protected Pose2d targetPose;
    private Supplier<Pose2d> goalPose;
    private ReefscapeJoystick joystick;
    private RobotState robotState;
    private DrivetrainBase drivetrainBase;
    private final ProfiledPIDController translationPID;
    private final ProfiledPIDController thetaController;
    private final double maxVelocity = 6;
    private Translation2d driverAdjustment;
    private final double ffMinDistance = 0.2;
    private final double ffMaxDistance = 0.8;
    private Translation2d lastSetpointTranslation;
    private final double loopTimeSec = 0.02;
    private Translation2d feedForward;
    private final double linearTolerance = 0.015;
    private final double thetaTolerance = Units.degreesToRadians(1.5);
    private Timer timer;
    private final SimpleTelemetry targetTelemetry = new SimpleTelemetry("targetPose");
    private final SimpleTelemetry profileTelemetry = new SimpleTelemetry("profilePose");


    /**
     * This command creates and follows a path.
     */
    public AutoAlign(DrivetrainBase drivetrainBase, ReefscapeJoystick joystick) {
        timer = new Timer();
        feedForward = new Translation2d();
        robotState = RobotState.getInstance();
        driverAdjustment = new Translation2d();
        this.joystick = joystick;

        this.drivetrainBase = drivetrainBase;
        translationPID = new ProfiledPIDController(2, 0.0, 0.1,
            new TrapezoidProfile.Constraints(3, 3));
        translationPID.setTolerance(linearTolerance);

        thetaController = new ProfiledPIDController(1.2, 0.0, 0.1,
            new TrapezoidProfile.Constraints(3, 3));
        thetaController.setTolerance(thetaTolerance);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

    }

    @Override
    public void initialize() {
        super.initialize();

        robotState.setTeleopAligning(true);
        timer.restart();
        goalPose = () -> {
            double linearMagnitude = 0;
            Translation2d linearVelocity = new Translation2d();

            // This can be called from auto. Right now there might not be a joystick in auto
            // so we need to skip driver input
            if (joystick != null) {
                linearMagnitude = MathUtil.applyDeadband(Math.hypot(joystick.getWpiX(),
                    joystick.getWpiY()), 0.1);
                linearVelocity = new Translation2d(joystick.getWpiX(),
                    joystick.getWpiY()).times(linearMagnitude);
            }

            Pose2d finalPose = new Pose2d(targetPose.getX(), targetPose.getY(), targetPose.getRotation());

            //scale by maxVelocity and then get movement in one periodic cycle
            feedForward = linearVelocity
                .times(maxVelocity);
//            calculates how long the robot will move in one interation
            driverAdjustment = feedForward.times(loopTimeSec);
            double distance = drivetrainBase.getPose().getTranslation()
                .minus(driverAdjustment)
                .getDistance(finalPose.getTranslation());

//            (distance - 0.3) makes adjustment sooner if number is smaller and later if number is bigger
//            dividing by 2.5 makes robot less sensitive to changes: increase to make less and decrease to make more
            double offsetT = MathUtil.clamp((distance - 0.3) / 2.5, 0.0, 1.0);
//            multiply by 1.75 causes correction to bigger: decrease to make corrections smaller
//            and vice versa. Decrease for less overshoot
//            return finalPose.transformBy(new Transform2d(0.0, offsetT * 1.0 , new Rotation2d()));
            return finalPose;
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
        super.execute();

        Pose2d currentPose = drivetrainBase.getPose();
        Pose2d currentGoalPose = goalPose.get();
        targetTelemetry.telemeterize(targetPose);
        profileTelemetry.telemeterize(currentGoalPose);

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
//        translationPID.reset(
//            lastSetpointTranslation.getDistance(currentGoalPose.getTranslation()),
//            translationPID.getSetpoint().velocity);

//        combine both feedforward and feedback to get drive velocity
        double driveVelocityScalar =
            MathUtil.clamp(
                translationPID.getSetpoint().velocity * ffScaler
                    + translationPID.calculate(distance, 0.0),
                -3, 3);
        Logger.recordOutput("AutoAlign/ff addition", translationPID.getSetpoint().velocity * ffScaler);
        Logger.recordOutput("AutoAlign/PID calculation", +translationPID.calculate(distance, 0.0));

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
            MathUtil.clamp(
                thetaController.getSetpoint().velocity * ffScaler
                    + thetaController.calculate(
                    currentPose.getRotation().getRadians(), currentGoalPose.getRotation().getRadians()),
                -3, 3);


//        we also add feedforward so the velocity of the actual robot changes not only the goal pose
//        when driver moves joystick
//        this transforms the drive velocity scalar into a vector.
//        By doing this we have a velocity is x and y
//        We use translation to change the speed to x and y velocity even though it is usually used for distance
        Translation2d driveVelocity =
            new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(currentGoalPose.getTranslation()).getAngle())
                .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
                .getTranslation();
//                .plus(feedForward);


        drivetrainBase.drive(new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity),
            true,1);

        Logger.recordOutput("AutoAlign/driveVeloicty", driveVelocity);
        Logger.recordOutput("AutoAlign/thetaVelocity", thetaVelocity);
        Logger.recordOutput("AutoAlign/ffScalar", ffScaler);
        Logger.recordOutput("AutoAlign/currentGoalPose", currentGoalPose);
        Logger.recordOutput("AutoAlign/currentDistance", distance);
        Logger.recordOutput("AutoAlign/driveVelocityScaler", driveVelocityScalar);
    }

    @Override
    public boolean isFinished() {
        boolean isFinished = false;
        boolean atTarget = translationPID.atGoal() && thetaController.atGoal();
//        boolean timerExpired = false;
        boolean timerExpired = timer.get() > 6;
//        boolean coralOut = !RobotState.getInstance().isCoralSensorTriggered();
        boolean coralOut = false;
        if (atTarget || timerExpired || coralOut || targetPose == null) {
            System.out.println("At Target:" + atTarget + " timerExpired:" + timerExpired + " coralOut " + coralOut);
            isFinished = true;
        }
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("auto align done");
        robotState.setTeleopAligning(false);
        drivetrainBase.drive(new ChassisSpeeds(), true);
        super.end(interrupted);
    }

    protected void setTargetPose(Pose2d pose) {
        targetPose = pose;
    }

}



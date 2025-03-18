package frc.robot.subsystems.stormnet;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.utils.StormSubsystem;
import org.littletonrobotics.junction.Logger;


public class StormNetSubsystem extends StormSubsystem {
    private StormNet stormNet;
    private RobotState state;
    private final double minDistance;
    private final double maxDistance;
    private final double maxRotationTheta;
    private final double lidarSeparation;
    private boolean isValid;
    private double offsetAngle;
    private double lidarHeading;

    public StormNetSubsystem() {
        StormNet.init();
        stormNet = StormNet.getInstance();

        if (Constants.Debug.debug) {
            stormNet.test();
        }

        minDistance = Constants.StormNet.minDistance;
        maxDistance = Constants.StormNet.maxDistance;
        maxRotationTheta = Math.toRadians(Constants.StormNet.maxRotation);
        lidarSeparation = Constants.StormNet.lidarSeparation;

        isValid = false;
        lidarHeading = 0;
        state = RobotState.getInstance();
    }

    @Override
    public void periodic() {
        super.periodic();

        if (state.isAprilTagDetected()) {
            calculateLidarRotation();
            state.setLidarAngles(lidarHeading, offsetAngle, isValid);
        } else {
            state.setLidarAngles(0.0, 0.0, false);
        }

        Logger.recordOutput("StormNet/LidarAngle", lidarHeading);
        Logger.recordOutput("StormNet/LidarIsValid", isValid);
    }

    private void calculateLidarRotation() {
        double[] distances = stormNet.getLidarDistances();
        int tag = state.getBestTag();

        if (tag < 0 ||
            distances[0] < minDistance ||
            distances[1] < minDistance ||
            distances[0] > maxDistance ||
            distances[1] > maxDistance) {
            isValid = false;
            lidarHeading = 0;
            return;
        }

        // We can see a tag and have a good reading. Calculate rotation error relative to the wall
        offsetAngle = Math.atan2(distances[0] - distances[1], lidarSeparation);

        Rotation2d tagHeading = FieldConstants.getPoseTag(tag).getRotation();
        Rotation2d robotHeading = tagHeading.plus(Rotation2d.fromRadians(offsetAngle));

        // Clamp lidarAngle to prevent extreme values
        if (Math.abs(robotHeading.getRadians() - tagHeading.getRadians()) < maxRotationTheta) {
            isValid = true;
            lidarHeading = robotHeading.getRadians();
        } else {
            isValid = false;
            lidarHeading = 0;
        }
    }
}

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.utils.vision.LimelightHelpers;

import java.util.Arrays;
import java.util.Optional;

public class StormLimelight {
    String limelightID;
    RobotState robotState;

    public StormLimelight(String limelightID) {
        this.limelightID = limelightID;
        this.robotState = RobotState.getInstance();
    }

    public void setGyroMeasurement(double yaw) {
        LimelightHelpers.SetRobotOrientation(limelightID, robotState.getYaw(), 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    public Optional<LimelightHelpers.PoseEstimate> getMT2Pose() {
        if (LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightID) != null) {
            return Optional.of(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightID));
        }
        return Optional.empty();
    }

    public double getDistance(int id) {
        Pose2d tagPose = FieldConstants.getPoseTag(id);

        Pose2d robotPose = robotState.getPose();
        return robotPose.minus(tagPose).getTranslation().getNorm();
    }

    public double getAvgDist() {
        LimelightHelpers.RawFiducial[] rawFiducials = getRawFiducials();
        double[] a = Arrays.stream(rawFiducials).mapToDouble(f -> f.id).toArray();
        double[] distances = new double[a.length];
        if (a.length == 0) {
            return 0;
        }
        for (int index = 0; index < a.length; index++) {
            distances[index] = getDistance((int) a[index]);
        }
        return Arrays.stream(distances).sum() / distances.length;
    }

    public boolean seesTag() {
        return LimelightHelpers.getTV(limelightID);
    }

    private LimelightHelpers.RawFiducial[] getRawFiducials () {
        return LimelightHelpers.getRawFiducials(limelightID);
    }

    public int tagsSeen() {
        if (seesTag()) {
            return getRawFiducials().length;
        } else {
            return 0;
        }
    }

}

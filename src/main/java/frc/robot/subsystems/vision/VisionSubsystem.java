package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.utils.StormSubsystem;
import frc.utils.vision.LimelightExtra;
import frc.utils.vision.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Optional;

import static frc.robot.Constants.Vision.*;
import static frc.utils.vision.LimelightHelpers.getRawFiducials;

public class VisionSubsystem extends StormSubsystem {
    private final RobotState robotState;
    private LimelightHelpers.LimelightResults latestLimelightResults = null;
    private int count;
    private Pose2d poseTag;
    private final double linearStdDevMegatag2Factor = 0.5;
    private final double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians
    private static double heading;
    StormLimelight limelightReef;
    StormLimelight[] limelights;
    static Pose2d estimatorPose;
    SwerveDrivePoseEstimator poseEstimator;
    int bestTag;

    public VisionSubsystem(SwerveDrivePoseEstimator swerveDrivePoseEstimator, StormLimelight... stormLimelights) {
        LimelightHelpers.setLEDMode_PipelineControl("");
        LimelightHelpers.setLEDMode_ForceBlink("");
        robotState = RobotState.getInstance();
        count = 0;
        poseTag = new Pose2d(12.64, 4.75, new Rotation2d(120));
        limelightReef = stormLimelights[0];
        limelights = stormLimelights;
        poseEstimator = swerveDrivePoseEstimator;


    }

    public Optional<LimelightHelpers.PoseEstimate> getMT2() {
//        if (getClosestLimelight().seesTag()) {
//            return getClosestLimelight().getMT2Pose();
//        } else {
//            return Optional.empty();
//        }
        if (LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightID) != null) {
            return Optional.of(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightID));
        }
        return Optional.empty();
    }

    public StormLimelight getClosestLimelight() {
        StormLimelight closestLimelight = null;
        double minAvgDist = Double.MAX_VALUE;

        for (StormLimelight limelight : limelights) {
            if (limelight.seesTag()) {
                double avgDist = limelight.getAvgDist();
                if (avgDist < minAvgDist) {
                    minAvgDist = avgDist;
                    closestLimelight = limelight;
                }
            }
        }

        return closestLimelight;
    }

    public boolean seesTag() {
//        if (getClosestLimelight() != null) {
//            return getClosestLimelight().seesTag();
//        } else {
//            return false;
//        }
        return LimelightHelpers.getTV(limelightID);
    }

    private double getDistance(int id) {
        Pose2d tagPose = FieldConstants.getPoseTag(id);

        Pose2d robotPose = estimatorPose;
        if (robotPose != null && tagPose != null) {
            return robotPose.minus(tagPose).getTranslation().getNorm();
        }
        return 0.0;
    }

    public double getAverageDistance() {
//        if (getClosestLimelight().seesTag()) {
//            return getClosestLimelight().getAvgDist();
//        } else {
//            return 0;
//        }
        if (estimatorPose != null && getRawFiducials(limelightID) != null) {
            LimelightHelpers.RawFiducial[] rawFiducials = getRawFiducials(limelightID);
            double[] a = Arrays.stream(rawFiducials).mapToDouble(f -> f.id).toArray();
            double[] distances = new double[a.length];
            if (a.length == 0) {
                return 0;
            }
            for (int index = 0; index < a.length; index++) {
                if (estimatorPose != null) {
                    distances[index] = getDistance((int) a[index]);
                }
            }
            return Arrays.stream(distances).sum() / distances.length;
        } else {
            return 0;
        }


    }

    private void setGyro(double headingDegrees) {
//        for (StormLimelight limelight : limelights) {
//            limelight.setGyroMeasurement(headingDegrees);
//        }
        LimelightHelpers.SetRobotOrientation(limelightID, headingDegrees, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    public int numOfTags() {
//        if (getClosestLimelight().seesTag()) {
//            return getClosestLimelight().tagsSeen();
//        } else {
//            return 0;
//        }
        return getRawFiducials(limelightID).length;
    }

    public int getBestTag() {
//        if (getClosestLimelight().seesTag()) {
//            return getClosestLimelight().getClosestTag();
//        } else {
//            return -1;
//        }
        return (int) LimelightExtra.getTID(limelightID);
    }

    public StormLimelight getLimelightFromID(String id) {
        StormLimelight targetLL = null;
        for (StormLimelight limelight : limelights) {
            if (limelight.limelightID.equals(id)) {
                targetLL = limelight;
            }
        }
        return targetLL;
    }

    public StormLimelight getLimelightFromIndex(int index) {
        try {
            return limelights[index];
        } catch (IndexOutOfBoundsException e) {
            return null;
        }
    }


    @Override
    public void periodic() {
        super.periodic();

        estimatorPose = poseEstimator.getEstimatedPosition();
        heading = estimatorPose.getRotation().getDegrees();
        setGyro(heading);
//        robotState.setTV(seesTag());
//        robotState.setIsVisionPoseValid(getMT2().isPresent());
//        robotState.setVisionPose(
//            getMT2().isPresent()
//                ? getMT2().get().pose
//                : null
//        );
        LimelightHelpers.RawFiducial[] rawFiducials = getRawFiducials(limelightID);
        if (rawFiducials.length > 0) {
            bestTag = rawFiducials[0].id;
        } else {
            bestTag = -1;
        }

        boolean rejectPose = false;
        Pose2d visionPose = null;
        double timeStamp = 0.0;
        LimelightHelpers.PoseEstimate poseEstimate = getMT2().orElse(null);
        if (poseEstimate != null) {

            rejectPose = !seesTag()
                || poseEstimate.pose.getX() <= 0.0
                || poseEstimate.pose.getY() <= 0.0;

            visionPose = poseEstimate.pose;
            timeStamp = poseEstimate.timestampSeconds;

            Logger.recordOutput("Reject Pose?", rejectPose);
            Logger.recordOutput("Vision/VisionPose", poseEstimate.pose);
        } else {
            Logger.recordOutput("Reject Pose?", rejectPose);
            Logger.recordOutput("Vision/VisionPose", new Pose2d());

        }
        if (!rejectPose) {
            double stdDevFactor = 0.0;

            // uncertainty grows quadratically as robot is farther away
            // more tags seen uncertainty is less
            if (getRawFiducials(limelightID).length > 0) {
                stdDevFactor = Math.pow(getAverageDistance(), 2.0) / getRawFiducials(limelightID).length;
            }
//        linearStdDevBaseline and angularStdDevBaseline:
//        base value for uncertainty then multiplied by factor above.
            double linearStdDev = linearStdDevBaseline * stdDevFactor;
            double angularStdDev = angularStdDevBaseline * stdDevFactor;

//        reduce uncertainty by half bc MegaTag2 is more stable
            linearStdDev *= linearStdDevMegatag2Factor;
//        MegaTag2 does not give rotation data (comes from gyro)
            angularStdDev *= angularStdDevMegatag2Factor;
            if (visionPose != null && robotState.getVisionEnabled()) {

                poseEstimator.addVisionMeasurement(visionPose, timeStamp,
                    VecBuilder.fill(linearStdDev, linearStdDev, 1e6));
            }
        }

    }
    public boolean isAligned() {
        if ((bestTag <= tagRightRed && bestTag >= tagLeftRed) || (bestTag >= tagLeftBlue && bestTag <=tagRightBlue)) {
            Pose2d left = FieldConstants.getReefTargetPose(FieldConstants.Side.LEFT, bestTag);
            Pose2d right = FieldConstants.getReefTargetPose(FieldConstants.Side.RIGHT, bestTag);
            if ((poseEstimator.getEstimatedPosition().minus(left).getTranslation().getNorm()<=0.01) || (poseEstimator.getEstimatedPosition().minus(right).getTranslation().getNorm()<=0.01)) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }
}

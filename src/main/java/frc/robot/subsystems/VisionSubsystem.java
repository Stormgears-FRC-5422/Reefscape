package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.StormLimelight;
import frc.utils.StormSubsystem;
import frc.utils.vision.LimelightExtra;
import frc.utils.vision.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Optional;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static frc.robot.Constants.Vision.limelightID;

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
    private static Rotation2d heading;
    StormLimelight limelightReef;
    StormLimelight[] limelights;
    static Pose2d estimatorPose;

    public VisionSubsystem(StormLimelight... stormLimelights) {
        LimelightHelpers.setLEDMode_PipelineControl("");
        LimelightHelpers.setLEDMode_ForceBlink("");
        robotState = RobotState.getInstance();
        count = 0;
        poseTag = new Pose2d(12.64, 4.75, new Rotation2d(120));
        limelightReef = stormLimelights[0];
        limelights = stormLimelights;

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
        return robotPose.minus(tagPose).getTranslation().getNorm();
    }

    public double getAverageDistance() {
//        if (getClosestLimelight().seesTag()) {
//            return getClosestLimelight().getAvgDist();
//        } else {
//            return 0;
//        }
        if (estimatorPose != null && LimelightHelpers.getRawFiducials(limelightID) !=null) {
            LimelightHelpers.RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(limelightID);
            double[] a = Arrays.stream(rawFiducials).mapToDouble(f -> f.id).toArray();
            double[] distances = new double[a.length];
            if (a.length == 0) {
                return 0;
            }
            for (int index = 0; index < a.length; index++) {
                distances[index] = getDistance((int) a[index]);
            }
            return Arrays.stream(distances).sum() / distances.length;
        } else {
            return 0;
        }


    }

    public void setGyro(double headingDegrees){
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
        return LimelightHelpers.getRawFiducials(limelightID).length;
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

    public static void setPoseestimatorPose(Pose2d poseestimatorPose) {
        estimatorPose = poseestimatorPose;
    }



    @Override
    public void periodic() {
        super.periodic();
        setGyro(heading.getDegrees());
//        robotState.setTV(seesTag());
//        robotState.setIsVisionPoseValid(getMT2().isPresent());
//        robotState.setVisionPose(
//            getMT2().isPresent()
//                ? getMT2().get().pose
//                : null
//        );
        //LimelightHelpers.SetRobotOrientation("", robotState.getYaw(), 0.0, 0.0, 0.0, 0.0, 0.0);

//        System.out.println(LimelightHelpers.getRawFiducials(limelightId)[0].id);
//        System.out.println(getAvgDist());
//        System.out.println(robotState.getYaw());
//            console(getMT2PoseEstimate().get().pose.toString());
//        Pose2d botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightId).pose;
//        if (botPose != null) {
//             Transform2d difference = poseTag.minus(botPose);
//           console(difference.toString());
//        }
//
//        Transform2d sigma = new Transform2d(poseTag, LimelightHelpers.getBotPose2d_wpiBlue(limelightId));
//        console("botPose in target space" + Arrays.toString(LimelightHelpers.getBotPose_TargetSpace(limelightId)));
//        RobotState.getInstance().setVisionPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight"),
//            LimelightHelpers.getTV("limelight"));

        boolean rejectPose = false;
        if (getMT2().isPresent()) {
            rejectPose =
                !seesTag()

                    || getMT2().get().pose.getX() < 0.0
                    || getMT2().get().pose.getY() < 0.0;

//            System.out.println("Empty? " + !LimelightHelpers.getTV(limelightId));

        }
        if (!rejectPose) {
            double stdDevFactor = 0.0;

            // uncertainty grows quadratically as robot is farther away
            // more tags seen uncertainty is less
            if (LimelightHelpers.getRawFiducials(limelightID).length>0) {
                stdDevFactor = Math.pow(getAverageDistance(), 2.0) /
                    LimelightHelpers.getRawFiducials(limelightID).length;
            }
//        linearStdDevBaseline and angularStdDevBaseline:
//        base value for uncertainty then multiplied by factor above.
            double linearStdDev = linearStdDevBaseline * stdDevFactor;
            double angularStdDev = angularStdDevBaseline * stdDevFactor;

//        reduce uncertainty by half bc MegaTag2 is more stable
            linearStdDev *= linearStdDevMegatag2Factor;
//        MegaTag2 does not give rotation data (comes from gyro)
            angularStdDev *= angularStdDevMegatag2Factor;
            if (getMT2().isPresent()) {
                robotState.addVisionMeasurments(getMT2().get().pose,
                    getMT2().get().timestampSeconds,
                    VecBuilder.fill(linearStdDev, linearStdDev, linearStdDev));
                Logger.recordOutput("Vision/VisionPose", getMT2().get().pose);
            }

        } else {
//            System.out.println("reject");
        }
    }

    public static void setHeading(Rotation2d rotation2d){
        heading = rotation2d;
    }

}

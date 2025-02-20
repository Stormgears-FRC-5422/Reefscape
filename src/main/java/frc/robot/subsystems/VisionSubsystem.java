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

public class VisionSubsystem extends StormSubsystem {
    private final RobotState robotState;
    private final String limelightId;
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

    public VisionSubsystem(String limelightId) {
        this.limelightId = limelightId;
        LimelightHelpers.setLEDMode_PipelineControl("");
        LimelightHelpers.setLEDMode_ForceBlink("");
        robotState = RobotState.getInstance();
        count = 0;
        poseTag = new Pose2d(12.64, 4.75, new Rotation2d(120));
        limelightReef = new StormLimelight(limelightId);

    }


    @Override
    public void periodic() {
        super.periodic();
        limelightReef.setGyroMeasurement(heading.getDegrees());
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
        if (limelightReef.getMT2Pose().isPresent()) {
            rejectPose =
                !LimelightHelpers.getTV(limelightId)

                    || limelightReef.getMT2Pose().get().pose.getX() < 0.0
                    || limelightReef.getMT2Pose().get().pose.getY() < 0.0;

//            System.out.println("Empty? " + !LimelightHelpers.getTV(limelightId));

        }
        if (!rejectPose) {
            double stdDevFactor = 0.0;

            // uncertainty grows quadratically as robot is farther away
            // more tags seen uncertainty is less
            if (limelightReef.seesTag()) {
                stdDevFactor = Math.pow(limelightReef.getAvgDist(), 2.0) /
                    limelightReef.tagsSeen();
            }
//        linearStdDevBaseline and angularStdDevBaseline:
//        base value for uncertainty then multiplied by factor above.
            double linearStdDev = linearStdDevBaseline * stdDevFactor;
            double angularStdDev = angularStdDevBaseline * stdDevFactor;

//        reduce uncertainty by half bc MegaTag2 is more stable
            linearStdDev *= linearStdDevMegatag2Factor;
//        MegaTag2 does not give rotation data (comes from gyro)
            angularStdDev *= angularStdDevMegatag2Factor;
            if (limelightReef.getMT2Pose().isPresent()) {
                robotState.addVisionMeasurments(limelightReef.getMT2Pose().get().pose,
                    limelightReef.getMT2Pose().get().timestampSeconds,
                    VecBuilder.fill(linearStdDev, linearStdDev, 1));
                Logger.recordOutput("Vision/VisionPose", limelightReef.getMT2Pose().get().pose);
            }

        } else {
//            System.out.println("reject");
        }
    }

    public double[] getCameraPose_TargetSpace() {
        return LimelightHelpers.getCameraPose_TargetSpace(limelightId);
    }
    public static void setHeading(Rotation2d rotation2d){
        heading = rotation2d;
    }

    public double getTX() {
        return LimelightExtra.getTX(limelightId);
    }

    public double getTY() {
        return LimelightExtra.getTY(limelightId);
    }

    public boolean getValid() {
        return LimelightExtra.hasValidTarget(limelightId);
    }

}

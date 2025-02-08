package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.utils.StormSubsystem;
import frc.utils.vision.LimelightExtra;
import frc.utils.vision.LimelightHelpers;

import java.util.Arrays;
import java.util.Optional;

import static edu.wpi.first.math.util.Units.degreesToRadians;

public class VisionSubsystem extends StormSubsystem {
    private final RobotState robotState;
    private final String limelightId;
    private LimelightHelpers.LimelightResults latestLimelightResults = null;
    private int count;
    private Pose2d poseTag;

    public VisionSubsystem(String limelightId) {
        this.limelightId = limelightId;
        LimelightHelpers.setLEDMode_PipelineControl("");
        LimelightHelpers.setLEDMode_ForceBlink("");
        robotState = RobotState.getInstance();
        count = 0;
        poseTag = new Pose2d(12.64,4.75, new Rotation2d(120));
    }

    private static Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) {
            //System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(degreesToRadians(inData[5]));
//        AprilTagPoseEstimator estimator = new AprilTagPoseEstimator();
        return new Pose2d(tran2d, r2d);
    }

    public LimelightHelpers.LimelightResults getLatestResults() {
        if (latestLimelightResults == null) {
            latestLimelightResults = LimelightHelpers.getLatestResults(limelightId);
            count += 1;
            if (count % 50 == 0) {
                //    System.out.println("info : " + latestLimelightResults.);
            }
        }
        return latestLimelightResults;
    }

    public void addDetectorDashboardWidgets(ShuffleboardTab layout) {
        layout.addBoolean("Target", () -> getLatestDetectorTarget().isPresent()).withPosition(0, 0);
        layout.addDouble("Tx", () -> {
            var optResults = getLatestDetectorTarget();
            if (optResults.isPresent()) {
                return optResults.get().tx;
            }
            return 0;
        }).withPosition(0, 1);
        layout.addDouble("Ty", () -> {
            var optResults = getLatestDetectorTarget();
            if (optResults.isPresent()) {
                return optResults.get().ty;
            }
            return 0;
        }).withPosition(0, 2);
        layout.addString("Class", () -> {
            var optResults = getLatestDetectorTarget();
            if (optResults.isPresent()) {
                return optResults.get().className;
            }
            return "";
        }).withPosition(0, 3);
    }

    public Optional<LimelightHelpers.LimelightTarget_Detector> getLatestDetectorTarget() {
        var results = getLatestResults();
        var targetResult = results;
//        System.out.println(results.valid);
//        System.out.println(Arrays.toString(results.targets_Detector));
//        System.out.println(results.targets_Detector.length);

        if (targetResult != null && targetResult.valid && targetResult.targets_Detector.length > 0) {
            return Optional.of(targetResult.targets_Detector[0]);
        }
        return Optional.empty();
    }

    public Optional<LimelightHelpers.LimelightTarget_Retro> getLatestRetroTarget() {
        var results = getLatestResults();
        if (results == null) {
            return Optional.empty();
        }
        var targetResult = results;
        if (targetResult != null && targetResult.valid && targetResult.targets_Retro.length > 0) {
            return Optional.of(targetResult.targets_Retro[0]);
        }
        return Optional.empty();
    }

    public Optional<LimelightHelpers.LimelightTarget_Fiducial> getLatestFiducialsTarget() {
        var results = getLatestResults();
        if (results == null) {
            return Optional.empty();
        }
        var targetResult = results;
        if (targetResult != null && targetResult.valid && targetResult.targets_Fiducials.length > 0) {
            return Optional.of(targetResult.targets_Fiducials[0]);
        }
        return Optional.empty();
    }
    public Optional<LimelightHelpers.LimelightTarget_Fiducial[]> getLatestFiducialsTargets() {
        var results = getLatestResults();
        if (results == null) {
            return Optional.empty();
        }
        var targetResult = results;
        if (targetResult != null && targetResult.valid && targetResult.targets_Fiducials.length > 0) {
            return Optional.of(targetResult.targets_Fiducials);
        }
        return Optional.empty();
    }

    public Optional<LimelightHelpers.PoseEstimate> getwpiBlue(){
        var results = getLatestResults();
        if (results == null) {
            return Optional.empty();
        }
        var targetResult = results;
        if (LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightId) != null) {
            return Optional.of(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightId));
        }
        return Optional.empty();
    }
    public double getDistance(int id) {
        Pose2d tagPose = getPoseTag(id);
        Pose2d robotPose = robotState.getPose();
        return robotPose.minus(tagPose).getTranslation().getNorm();
    }
    public double getAvgDist() {
        double[] a = LimelightExtra.getTagIDs(limelightId);
        double[] distances = new double[a.length];
        for (double i : a) {
            distances[(int) i] = getDistance((int) i);
        }
        return Arrays.stream(distances).sum() / distances.length;
    }
    public Pose2d getTargetPose(String side, int id){
        if (side == "left"){
            return getPoseTag(id).transformBy(new Transform2d(new Translation2d(-6.5, 0), new Rotation2d(0)));
        } else if (side == "right"){
            return getPoseTag(id).transformBy(new Transform2d(new Translation2d(6.5, 0), new Rotation2d(0)));
        } else {
            return null;
        }
    }
    public Pose2d getPoseTag(int id) {
        switch (id) {
            case 1:
                return new Pose2d(Constants.Vision.tag1X, Constants.Vision.tag1Y, new Rotation2d(Constants.Vision.tag1Rotation));
            case 2:
                return new Pose2d(Constants.Vision.tag2X, Constants.Vision.tag2Y, new Rotation2d(Constants.Vision.tag2Rotation));
            case 3:
                return new Pose2d(Constants.Vision.tag3X, Constants.Vision.tag3Y, new Rotation2d(Constants.Vision.tag3Rotation));
            case 4:
                return new Pose2d(Constants.Vision.tag4X, Constants.Vision.tag4Y, new Rotation2d(Constants.Vision.tag4Rotation));
            case 5:
                return new Pose2d(Constants.Vision.tag5X, Constants.Vision.tag5Y, new Rotation2d(Constants.Vision.tag5Rotation));
            case 6:
                return new Pose2d(Constants.Vision.tag6X, Constants.Vision.tag6Y, new Rotation2d(Constants.Vision.tag6Rotation));
            case 7:
                return new Pose2d(Constants.Vision.tag7X, Constants.Vision.tag7Y, new Rotation2d(Constants.Vision.tag7Rotation));
            case 8:
                return new Pose2d(Constants.Vision.tag8X, Constants.Vision.tag8Y, new Rotation2d(Constants.Vision.tag8Rotation));
            case 9:
                return new Pose2d(Constants.Vision.tag9X, Constants.Vision.tag9Y, new Rotation2d(Constants.Vision.tag9Rotation));
            case 18:
                return new Pose2d(Constants.Vision.tag18X, Constants.Vision.tag18Y, new Rotation2d(Constants.Vision.tag18Rotation));
            case 19:
                return new Pose2d(Constants.Vision.tag19X, Constants.Vision.tag19Y, new Rotation2d(Constants.Vision.tag19Rotation));
            case 20:
                return new Pose2d(Constants.Vision.tag20X, Constants.Vision.tag20Y, new Rotation2d(Constants.Vision.tag20Rotation));
            case 21:
                return new Pose2d(Constants.Vision.tag21X, Constants.Vision.tag21Y, new Rotation2d(Constants.Vision.tag21Rotation));
            case 22:
                return new Pose2d(Constants.Vision.tag22X, Constants.Vision.tag22Y, new Rotation2d(Constants.Vision.tag22Rotation));
        }
    }


    @Override
    public void periodic() {
        super.periodic();
        latestLimelightResults = null;
        //Pose2d botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightId).pose;
        //if (botPose != null) {
       //     Transform2d difference = poseTag.minus(botPose);
         //   console(difference.toString());
        //}

//        Transform2d sigma = new Transform2d(poseTag, LimelightHelpers.getBotPose2d_wpiBlue(limelightId));
        //console("botPose in target space" + Arrays.toString(LimelightHelpers.getBotPose_TargetSpace(limelightId)));
//        RobotState.getInstance().setVisionPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight"),
//            LimelightHelpers.getTV("limelight"));

    }
    public double[] getCameraPose_TargetSpace(){
        return LimelightHelpers.getCameraPose_TargetSpace(limelightId);
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

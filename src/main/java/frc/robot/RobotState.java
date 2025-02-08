package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.vision.LimelightHelpers;

import static edu.wpi.first.math.util.Units.degreesToRadians;

public class RobotState extends SubsystemBase {
    public enum StateAlliance {
        RED, BLUE, MISSING
    }

    public enum StatePeriod {
        NONE, DISABLED, AUTONOMOUS, TELEOP, TEST
    }

    private static RobotState m_instance;
    private StateAlliance m_alliance = StateAlliance.MISSING;
    private Pose2d currentPose = new Pose2d();
    private VisionMeasurement visionMeasurement = null;

    private StatePeriod m_period = StatePeriod.NONE;
    private boolean m_didAuto = false;
    private boolean m_didTeleop = false;

    public static RobotState getInstance() {
        if (m_instance != null) return m_instance;

        m_instance = new RobotState();
        return m_instance;
    }

    public void setAlliance(StateAlliance alliance) {
        if (m_alliance != alliance) {
            System.out.println("**********");
            System.out.println("Alliance is now reported as " + alliance + ", was " + m_alliance);
            System.out.println("**********");
        }
        m_alliance = alliance;
    }

    public void setPeriod(StatePeriod period) {
        m_period = period;

        switch (period) {
            case AUTONOMOUS -> m_didAuto = true;
            case TELEOP -> m_didTeleop = true;
        }
    }

    public StatePeriod getPeriod() {
        return m_period;
    }

    public boolean getDidAuto() {
        return m_didAuto;
    }

    public boolean getDidTeleop() {
        return m_didTeleop;
    }

    public StateAlliance getAlliance() {
        return m_alliance;
    }

    public boolean isAllianceBlue() {
        return m_alliance == StateAlliance.BLUE;
    }

    public boolean isAllianceRed() {
        return m_alliance == StateAlliance.RED;
    }

    public boolean isAllianceMissing() {
        return m_alliance == StateAlliance.MISSING;
    }

    public void setHeading(Rotation2d angle) {
        currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d(angle.getRadians()));
    }

    public Rotation2d getHeading() {
        return currentPose.getRotation();
    }

    public void setPose(Pose2d pose) {
        // Make a copy, not a reference to the same object!
        currentPose = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getRotation().getRadians()));
    }

    public Pose2d getPose() {
        return currentPose;
    }

    public boolean isVisionPoseValid() {
        return LimelightHelpers.getTV(Constants.Vision.limelightID);
    }

    public Pose2d getVisionPose() {
//        Optional<LimelightHelpers.LimelightTarget_Fiducial> visionResult = vision.getLatestFiducialsTarget();
//        return toPose2D(visionResult.map(limelightTarget_fiducial -> limelightTarget_fiducial.botpose_wpiblue).orElse(null));
        return LimelightHelpers.getBotPose2d_wpiBlue(Constants.Vision.limelightID);
    }

    public void addVisionMeasurments(Pose2d visionRobotPoseMeters,
                                     double timestampSeconds,
                                     Matrix<N3, N1> visionMeasurementStdDevs) {
        visionMeasurement = new VisionMeasurement(visionRobotPoseMeters,
            timestampSeconds, visionMeasurementStdDevs);

    }

    public VisionMeasurement getVisionMeasurments() {
        return  visionMeasurement;
    }

    private static Pose2d toPose2D(double[] inData) {
        if (inData == null || inData.length < 6) {
            //System.err.println("Bad LL 2D Pose Data!");
            return null;
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }


    public record VisionMeasurement(Pose2d visionRobotPoseMeters,
                                    double timestampSeconds,
                                    Matrix<N3, N1> visionMeasurementStdDevs) {
    }
    public boolean isTagDetected() {
        return LimelightHelpers.getTV(Constants.Vision.limelightID);
    }
}




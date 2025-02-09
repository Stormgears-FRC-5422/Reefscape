package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.vision.LimelightHelpers;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import static edu.wpi.first.math.util.Units.degreesToRadians;

public class RobotState extends SubsystemBase {
    public enum StateAlliance {
        RED, BLUE, MISSING
    }

    public enum StatePeriod {
        NONE, DISABLED, AUTONOMOUS, TELEOP, TEST
    }

    public enum StateSimMode {
        REAL, SIMULATION, AKIT_REPLAY, AKIT_SIM;
    }

    private static RobotState m_instance;
    // TODO - this should be set via updateAlliance()
    //  private StateAlliance m_alliance = StateAlliance.MISSING;
    private StateAlliance m_alliance = StateAlliance.BLUE;
    private Pose2d currentPose = new Pose2d();
    //private Pose2d visionPose = new Pose2d();

    private StatePeriod m_period = StatePeriod.NONE;
    private final StateSimMode m_simMode;
    private boolean m_didAuto = false;
    private boolean m_didTeleop = false;
    boolean m_isCoralSensorTriggered = false;
    private boolean climberHasBeenHomed = false;

    // Call createInstance from robotInit()
    public static RobotState createInstance() {
        if (m_instance != null) return m_instance;

        m_instance = new RobotState();
        return m_instance;
    }

    public static RobotState getInstance() {
        return m_instance;
    }

    private RobotState() {
        if (RobotBase.isReal()) {
            m_simMode = StateSimMode.REAL;
        } else if (Constants.Toggles.useAdvantageKit) {
            if (Constants.Akit.doReplay) {
                m_simMode = StateSimMode.AKIT_REPLAY;
            } else {
                m_simMode = StateSimMode.AKIT_SIM;
            }
        } else { // basic simulation
            m_simMode = StateSimMode.SIMULATION;            ;
        }
    }

    public StateSimMode getSimMode() {
        return m_simMode;
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

    public void setClimberHasBeenHomed(boolean hasBeenHomed) {
        climberHasBeenHomed = hasBeenHomed;
    }

    public boolean climberHasBeenHomed() {
        return climberHasBeenHomed;
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

    private static Pose2d toPose2D(double[] inData) {
        if (inData == null || inData.length < 6) {
            //System.err.println("Bad LL 2D Pose Data!");
            return null;
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

    public void setCoralSensorTriggered(boolean triggered) {
        m_isCoralSensorTriggered = triggered;
    }

    public boolean isCoralSensorTriggered() {
        return m_isCoralSensorTriggered;
    }

    // TODO: code this (Meher)
    public boolean isElevatorHomed() {
        return true;
    }

    // TODO: code this (Raghav)
    public boolean isAprilTagDetected() {
        return false;
    }
}

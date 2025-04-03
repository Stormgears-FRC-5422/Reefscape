package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.misc.BatteryMonitor.BatteryState;
import frc.robot.subsystems.onElevator.Elevator.ElevatorState;


import static edu.wpi.first.math.util.Units.degreesToRadians;

public class RobotState extends SubsystemBase {
    public enum StateAlliance {
        RED, BLUE, MISSING
    }

    public enum StatePeriod {
        NONE, DISABLED, AUTONOMOUS, TELEOP, TEST
    }
    private double yawDouble;

    public enum StateSimMode {
        REAL, SIMULATION, AKIT_REPLAY, AKIT_SIM;
    }

    private static RobotState m_instance;
    // TODO - this should be set via updateAlliance()
    //  private StateAlliance m_alliance = StateAlliance.MISSING;
    private StateAlliance m_alliance = StateAlliance.BLUE;
    private Pose2d currentPose = new Pose2d();
    private VisionMeasurement visionMeasurement = null;

    private StatePeriod m_period = StatePeriod.NONE;
    private final StateSimMode m_simMode;
    private boolean m_didAuto = false;
    private boolean m_didTeleop = false;

    boolean m_isCoralSensorTriggered = false;
    private boolean elevatorHasBeenHomed = false;
    private boolean intakeWristHasBeenHomed = false;
    private boolean algaeIntakeHasBeenHomed = false;
    private ElevatorState elevatorState = ElevatorState.UNKNOWN;
    private boolean isAligned = false;
    private boolean isTeleopAligning = false;
    private boolean isVisionPoseValid = false;
    private Pose2d MT2Pose;
    private boolean isAprilTagDetected;
    private boolean visionEnabled = true;

    private boolean m_joystickAndButtonBoardConfigured = false;

    private BatteryState batteryState;
    int bestTag;
    private double lidarAngle;
    private boolean lidarAngleIsValid;
    private double lidarOffsetAngle;
    private boolean isClimberFullyForward;
    private boolean isClimberLockedIn;
    private boolean climberHasTriggered;
    private boolean didAllianceUpdated = false;
    private RobotState.StateAlliance lastAlliance = StateAlliance.MISSING;



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

        this.batteryState = BatteryState.GOOD;
    }

    public boolean isClimberFullyForward() {
        return isClimberFullyForward;
    }

    public void setClimberFullyForward(boolean isClimberFullyForward){
        this.isClimberFullyForward = isClimberFullyForward;
    }

    public boolean isClimberLockedIn(){
        return isClimberLockedIn;
    }

    public void setClimberLockedIn(boolean isClimberLockedIn){
        this.isClimberLockedIn = isClimberLockedIn;
    }

    public void setClimberHasTriggered(boolean climberHasTriggered){
        this.climberHasTriggered = climberHasTriggered;
    }

    public boolean getClimberHasTriggered(){
        return climberHasTriggered;
    }

    public boolean isJoystickAndButtonBoardConfigured() {
        return m_joystickAndButtonBoardConfigured;
    }

    public void setJoystickAndButtonBoardConfigured(boolean state) {
        m_joystickAndButtonBoardConfigured=state;
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

    public void setLastAlliance(){
        lastAlliance = getAlliance();
    }

    public boolean didAllianceChange(){
        return getAlliance()!=lastAlliance;
    }

    public boolean isAllianceRed() {
        return m_alliance == StateAlliance.RED;
    }

    public boolean didAllianceUpdated(){
        return didAllianceUpdated;
    }

    public void setAllianceUpdated(boolean completed){
        didAllianceUpdated = completed;
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

    public void setElevatorHasBeenHomed(boolean hasBeenHomed) {
        elevatorHasBeenHomed = hasBeenHomed;
    }

    public boolean elevatorHasBeenHomed() {
        return elevatorHasBeenHomed;
    }

    public ElevatorState getElevatorState() {
        return elevatorState;
    }
    public void setElevatorState(ElevatorState state) {
        elevatorState = state;
    }

    public void setPose(Pose2d pose) {
        // Make a copy, not a reference to the same object!
        currentPose = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getRotation().getRadians()));
    }

    public Pose2d getPose() {
        return currentPose;
    }

    public boolean isVisionPoseValid() {
        return isVisionPoseValid;
    }

    public void setIsVisionPoseValid(boolean valid) {
        isVisionPoseValid = valid;
    }

    public Pose2d getVisionPose() {
        return MT2Pose;
    }
    public void setVisionPose(Pose2d pose){
        MT2Pose = pose;
    }

    public void addVisionMeasurments(Pose2d visionRobotPoseMeters,
                                     double timestampSeconds,
                                     Matrix<N3, N1> visionMeasurementStdDevs) {
        Pose2d tempPose = new Pose2d(visionRobotPoseMeters.getTranslation(), visionRobotPoseMeters.getRotation());
        visionMeasurement = new VisionMeasurement(tempPose,
            timestampSeconds, visionMeasurementStdDevs);

    }

    public VisionMeasurement getVisionMeasurment() {
        return visionMeasurement;
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
    public boolean isAprilTagDetected() {
        return isAprilTagDetected;
    }
    public void setTV(boolean tv) {
        isAprilTagDetected = tv;
    }
    public void setYaw(double yaw) {
        yawDouble = yaw;
    }
    public double getYaw() {
        return yawDouble;
    }

    public void setCoralSensorTriggered(boolean triggered) {
        m_isCoralSensorTriggered = triggered;
    }

    public void setIntakeWristHasBeenHomed(boolean homed) {
        intakeWristHasBeenHomed = homed;
    }

    public boolean getIntakeWristHasBeenHomed() {
        return intakeWristHasBeenHomed;
    }

    public void setAlgaeIntakeHasBeenHomed(boolean homed) {
        algaeIntakeHasBeenHomed = homed;
    }

    public boolean getAlgaeIntakeHasBeenHomed() {
        return algaeIntakeHasBeenHomed;
    }

    public boolean isCoralSensorTriggered() {
        return m_isCoralSensorTriggered;
    }


    public boolean isAligned(){
        return isAligned;
    }

    public void setAligned(boolean aligned){
        isAligned = aligned;
    }

    public boolean isTeleopAligning(){
        return isTeleopAligning;
    }

    public void setTeleopAligning(boolean aligning){
        isTeleopAligning = aligning;
    }

    public boolean getVisionEnabled(){
        return visionEnabled;
    }

    public boolean allowEndgame() {
        if (DriverStation.isFMSAttached()) {
            return DriverStation.getMatchTime() < Constants.Game.endGameTime;
        } else {
            return true;
        }
    }

    public void setBatteryState(BatteryState batteryState) {
        this.batteryState = batteryState;
    }

    public BatteryState getBatteryState() {
        return this.batteryState;
    }

    public void setBestTag(int bestTag) {
        this.bestTag = bestTag;
    }

    public int getBestTag() {
        return bestTag;
    }

    public void setLidarAngles(double lidarHeading, double lidarOffsetAngle, boolean isValid) {
        this.lidarAngle = lidarHeading;
        this.lidarAngleIsValid = isValid;
        this.lidarOffsetAngle = lidarOffsetAngle;
    }

    public double getLidarAngle() {
        return lidarAngle;
    }

    public boolean getLidarAngleIsValid() {
        return lidarAngleIsValid;
    }

    public double getLidarOffsetAngle() { return lidarOffsetAngle; }

}

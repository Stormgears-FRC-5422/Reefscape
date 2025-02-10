package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.SparkConstants;
import frc.utils.StormSubsystem;

import frc.robot.RobotState;



public class Elevator extends StormSubsystem {

    public enum ElevatorLevel {
        UNKNOWN(Double.NaN),
        FLOOR(Double.NEGATIVE_INFINITY),
        HOME(0),
        BOTTOM(1),
        STORE(Double.NaN),
        LEVEL1(Double.NaN),
        LEVEL2(Double.NaN),
        LEVEL3(Double.NaN),
        LEVEL4(Double.NaN),
        TOP(20),
        CEILING(Double.POSITIVE_INFINITY);

        private double position;

        ElevatorLevel(double position) {
            this.position = position;
        }

        public double getValue() {
            return position;
        }
    }
    public enum ElevatorState {
        UNKNOWN,
        HOMING,
        HOME,
        FLOOR,
        BOTTOM,
        STORE,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        LEVEL4,
        TOP,
        CEILING;
    }

    private double elevatorSpeed;
    private boolean hasBeenHomed = false;
    private RelativeEncoder m_followerEncoder;
    public RelativeEncoder m_leaderEncoder;

    private final RobotState robotState;

    private SparkLimitSwitch topLimitSwitch;
    private SparkLimitSwitch bottomLimitSwitch;

    private final SparkMax elevatorLeader;
    private final SparkMax elevatorFollower;

    private ElevatorLevel targetLevel;
    private ElevatorState currentState;

    private boolean softLimitsEnabled = true;
    private SparkMaxConfig globalConfig = new SparkMaxConfig();



    public Elevator() {
        robotState = RobotState.getInstance();


        elevatorLeader = new SparkMax(14, SparkLowLevel.MotorType.kBrushless);
        elevatorFollower = new SparkMax(15, SparkLowLevel.MotorType.kBrushless);

        m_leaderEncoder = elevatorLeader.getEncoder();

        m_followerEncoder = elevatorFollower.getEncoder();

        topLimitSwitch = elevatorLeader.getForwardLimitSwitch();
        bottomLimitSwitch = elevatorLeader.getReverseLimitSwitch();

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(SparkConstants.CurrentLimit);
        globalConfig.idleMode(Constants.Elevator.brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

        globalConfig.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);

        globalConfig.softLimit
        .forwardSoftLimit(ElevatorLevel.TOP.getValue())
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(ElevatorLevel.BOTTOM.getValue())
        .reverseSoftLimitEnabled(true);

        SparkMaxConfig elevatorLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();

        elevatorLeaderConfig.apply(globalConfig).inverted(true);
        elevatorFollowerConfig.apply(globalConfig).follow(elevatorLeader, true);

        elevatorLeader.configure(elevatorLeaderConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        elevatorFollower.configure(elevatorFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        targetLevel = ElevatorLevel.UNKNOWN;
        currentState = ElevatorState.UNKNOWN;
    }

    @Override
    public void periodic() {
        super.periodic();

        double currentPosition = m_leaderEncoder.getPosition();
        if (robotState.getPeriod() != RobotState.StatePeriod.DISABLED) {
            console("position is " + currentPosition + ", target is " + targetLevel.getValue(), 50);
        }

        if (elevatorSpeed != 0) {
            if (currentPosition < targetLevel.getValue()) {
                moveElevatorUp();
            } else if (currentPosition > targetLevel.getValue()) {
                moveElevatorDown();
            } else {
                moveElevatorNot();
            }
        }

        Logger.recordOutput("Elevator/LeaderEncoder", m_followerEncoder.getPosition());
        Logger.recordOutput("Elevator/TargetLevel", targetLevel);
    }

    public void setTargetLevel(ElevatorLevel targetLevel) {
        this.targetLevel = targetLevel;
    }

    public void setCurrentState(ElevatorState state) {
        this.currentState = state;

        switch (state) {
            case UNKNOWN:
                hasBeenHomed = false;
                robotState.setElevatorHasBeenHomed(hasBeenHomed);
                break;

            case HOMING:
                disableSoftLimits();
                break;

            case HOME:
                console("***** ELEVATOR IS NOW HOME *****");
                hasBeenHomed = true;
                robotState.setElevatorHasBeenHomed(hasBeenHomed);
                m_leaderEncoder.setPosition(ElevatorLevel.HOME.getValue());
                m_followerEncoder.setPosition(ElevatorLevel.HOME.getValue());
                enableSoftLimits();
                break;
        }
    }

    public void setSpeed(double speed) {
        this.elevatorSpeed = Math.abs(speed);
    }

    public ElevatorLevel getTargetLevel() {
        return targetLevel;
    }

    private void moveElevatorUp() {
        elevatorLeader.set(elevatorSpeed);
    }

    private void moveElevatorDown() {
        elevatorLeader.set(-elevatorSpeed);
    }

    private void moveElevatorNot() {
        elevatorLeader.set(0);
    }

    public boolean isAtHome() {
        double encoderPosition = m_leaderEncoder.getPosition();
        return encoderPosition >= -1 && encoderPosition <= 1;
        // return bottomLimitSwitch.isPressed();
    }

    public boolean isatCeiling() {
        return topLimitSwitch.isPressed();
    }

    public void stopElevator() {
        setSpeed(0);
        elevatorLeader.set(0);
    }

    public void enableSoftLimits() {
        softLimitsEnabled = true;

        globalConfig.softLimit
            .forwardSoftLimit(ElevatorLevel.TOP.getValue())
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(ElevatorLevel.BOTTOM.getValue())
            .reverseSoftLimitEnabled(true);

//        elevatorLeader.configure(globalConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
//        elevatorFollower.configure(globalConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        elevatorLeader.configure(globalConfig, SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters);
        elevatorFollower.configure(globalConfig, SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters);
    }
    public void disableSoftLimits() {
        softLimitsEnabled = false;

        globalConfig.softLimit
            .forwardSoftLimit(ElevatorLevel.TOP.getValue())
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimit(ElevatorLevel.BOTTOM.getValue())
            .reverseSoftLimitEnabled(false);

//        elevatorLeader.configure(globalConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
//        elevatorFollower.configure(globalConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        elevatorLeader.configure(globalConfig, SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters);
        elevatorFollower.configure(globalConfig, SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters);
    }

    public boolean hasElevatorBeenHomed() {
        return hasBeenHomed;
    }
}

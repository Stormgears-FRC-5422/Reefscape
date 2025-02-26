package frc.robot.subsystems.onElevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.Constants;
import frc.robot.Constants.SparkConstants;
import frc.robot.RobotState;
import frc.utils.StormSubsystem;
import org.littletonrobotics.junction.Logger;

public class Elevator extends StormSubsystem {

    private final RobotState robotState;
    private final SparkMax elevatorLeader;
    private final SparkMax elevatorFollower;
    private final RelativeEncoder leaderEncoder;
    private final SparkClosedLoopController controller;
    private final ElevatorFeedforward feedForward;

    private double elevatorSpeed;
    private boolean hasBeenHomed = false;

    private double targetPosition;
    private double currentPosition;
    private ElevatorState currentState;
    private SparkMaxConfig elevatorLeaderConfig = new SparkMaxConfig();

    public Elevator() {
        robotState = RobotState.getInstance();

        elevatorLeader = new SparkMax(Constants.Elevator.leaderID, SparkLowLevel.MotorType.kBrushless);
        elevatorFollower = new SparkMax(Constants.Elevator.followerID, SparkLowLevel.MotorType.kBrushless);

        leaderEncoder = elevatorLeader.getEncoder();

        elevatorLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();

        globalConfig.smartCurrentLimit(SparkConstants.CurrentLimit)
            .voltageCompensation(SparkConstants.NominalVoltage)
            .idleMode(Constants.Elevator.brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

        elevatorLeaderConfig.apply(globalConfig)
            .inverted(Constants.Elevator.invertLeader);

        // Hard limits
        elevatorLeaderConfig.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyOpen)
            .forwardLimitSwitchEnabled(false)
            .reverseLimitSwitchType(Type.kNormallyOpen)
            .reverseLimitSwitchEnabled(false);

        // Soft limits
        elevatorLeaderConfig.softLimit
            .forwardSoftLimit(ElevatorLevel.TOP.getValue())
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(ElevatorLevel.BOTTOM.getValue())
            .reverseSoftLimitEnabled(true);

        double kP = Constants.Elevator.kP ; // volts/rot 0.5 - 1
        double kI = Constants.Elevator.kI;  // volts/ rot*s  0.01 - 0.1
        double kD = Constants.Elevator.kD;  // volts / rot/s  0.001 - 0.01

        double maxV = Constants.Elevator.maxV;  // volts
        double maxVPct = maxV / SparkConstants.NominalVoltage; // percentage (-1 to 1)

        // These functions can optionally take a slot - e.g. ClosedLoopSlot.kSlot0 is the default
        elevatorLeaderConfig.closedLoop
            .p(kP)
            .i(kI)
            .d(kD)
            .outputRange(-maxVPct, maxVPct);

        elevatorLeaderConfig.closedLoopRampRate(Constants.Elevator.closedLoopRampRate);

        elevatorLeader.configure(elevatorLeaderConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        elevatorFollowerConfig.apply(globalConfig)
            .follow(elevatorLeader, true);
        elevatorFollower.configure(elevatorFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        double kS = Constants.Elevator.kS;  // volts
        double kG = Constants.Elevator.kG;  // volts
        double kV = Constants.Elevator.kV;  // volts / rot/s
        double kA = Constants.Elevator.kA;  // negligible - ignore - 0.032 volts / rot/s
        feedForward = new ElevatorFeedforward(kS, kG, kV, kA);

        controller = elevatorLeader.getClosedLoopController();
        setState(ElevatorState.UNKNOWN);
    }

    @Override
    public void periodic() {
        super.periodic();

        currentPosition = leaderEncoder.getPosition();
        double ffVoltage = 0;

//        if (Constants.Debug.debug && robotState.getPeriod() != RobotState.StatePeriod.DISABLED) {
//            console("currentPosition = " + currentPosition, 50);
//        }

        switch (currentState) {
            case HOMING:
                home();
                elevatorLeader.set(elevatorSpeed);
                break;

            case SIMPLE_MOTION:
                if (hasBeenHomed) {
                    simpleMove(currentPosition);
                    elevatorLeader.set(elevatorSpeed);
                } else {
                    console("Stubbornly refusing to move before homed!", 25);
                }
                break;

            case PID_MOTION:
                if (hasBeenHomed) {
                    ffVoltage = feedForward.calculate(0); // 0 here basically gives us gravity compensation
                    controller.setReference(targetPosition,
                        SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVoltage);
                } else {
                    console("Stubbornly refusing to move before homed!", 25);
                }
                break;

            default:
                elevatorLeader.set(0);
        }

        Logger.recordOutput("Elevator/LeaderEncoder", currentPosition);
        Logger.recordOutput("Elevator/LeaderVelocity", leaderEncoder.getVelocity());
        Logger.recordOutput("Elevator/TargetLevel", targetPosition);
        Logger.recordOutput("Elevator/Feedforward", ffVoltage);
    }

    public void setState(ElevatorState state) {
        this.currentState = state;
        robotState.setElevatorState(state);

        switch (state) {
            case UNKNOWN:
                console("***** UNKNOWN state *****");
                hasBeenHomed = false;
                robotState.setElevatorHasBeenHomed(hasBeenHomed);
                enableSoftLimits(true);
                targetPosition = Double.NaN;
                break;

            case HOMING:
                console("***** HOMING state *****");
                enableSoftLimits(false);
                break;

            case HOME:
                console("***** HOME state *****");
                hasBeenHomed = true;
                robotState.setElevatorHasBeenHomed(hasBeenHomed);
                leaderEncoder.setPosition(ElevatorLevel.HOME.getValue());
                enableSoftLimits(true);
                break;

            case SIMPLE_MOTION:
                console("***** SIMPLE_MOTION state *****");
                break;

            case PID_MOTION:
                console("***** PID_MOTION state *****");
                break;

            case IDLE:
                console("***** IDLE state *****");
                targetPosition = Double.NaN;
                stopElevator();
                break;
        }
    }

    public boolean isAtHome() {
        if (Constants.Elevator.useCurrentLimitHomeStrategy) {
            // might want to use an average here to minimize spikes
            console("isAtHome output current: " + elevatorLeader.getOutputCurrent());
            return elevatorLeader.getOutputCurrent() > Constants.Elevator.stallCurrentLimit;
        } else {
            console("isAtHome current position: " + currentPosition);
            return Math.abs(currentPosition) < Constants.Elevator.homePositionThreshold;
        }
    }

    private void home() {
        if (Constants.Elevator.useCurrentLimitHomeStrategy) {
            // might want to use an average here to minimize spikes
            setSpeed(Constants.Elevator.stallCurrentSpeed);
        } else {
            setSpeed(0.0);
        }
    }

    public boolean isAtTarget() {
        return Math.abs(currentPosition - targetPosition) < Constants.Elevator.allowedError;
    }


    private void simpleMove(double currentPosition) {
        if (currentPosition < targetPosition) {
            setSpeed(Constants.Elevator.simpleUpSpeed);
        } else if (currentPosition > targetPosition) {
            setSpeed(-Constants.Elevator.simpleDownSpeed);
        } else {
            setSpeed(0);
        }
    }

    private void setSpeed(double speed) {
        this.elevatorSpeed = speed;
    }

    public void setTargetLevel(ElevatorLevel targetLevel) {
        setTargetPosition(targetLevel.getValue());
    }

    public void setTargetPosition(double position) {
        this.targetPosition = position;
    }

    public double getCurrentPosition() {return currentPosition;}

    public void stopElevator() {
        setSpeed(0);
        // This should be redundant, but for safety stop ASAP
        elevatorLeader.set(0);
    }

    public void enableSoftLimits(boolean enable) {
        elevatorLeaderConfig.softLimit
            .forwardSoftLimit(ElevatorLevel.TOP.getValue())
            .forwardSoftLimitEnabled(enable)
            .reverseSoftLimit(ElevatorLevel.BOTTOM.getValue())
            .reverseSoftLimitEnabled(enable);

        elevatorLeader.configure(elevatorLeaderConfig, SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters);
    }

    public enum ElevatorLevel {
        UNKNOWN(Double.NaN),
        FLOOR(Double.NEGATIVE_INFINITY),
        HOME(Constants.Elevator.home),
        BOTTOM(Constants.Elevator.bottom),
        LEVEL1(Constants.Elevator.level1),
        LEVEL2(Constants.Elevator.level2),
        LEVEL3(Constants.Elevator.level3),
        LEVEL4(Constants.Elevator.level4),
        TOP(Constants.Elevator.top),
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
        IDLE,
        SIMPLE_MOTION,
        PID_MOTION,
        HOLD,
    }
}

package frc.robot.subsystems;

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
    private final RelativeEncoder followerEncoder;
    private final SparkClosedLoopController controller;
    private final ElevatorFeedforward feedForward;

    private double elevatorSpeed;
    private boolean hasBeenHomed = false;

    private ElevatorLevel targetLevel;
    private ElevatorState currentState;
    private SparkMaxConfig elevatorLeaderConfig = new SparkMaxConfig();

    public Elevator() {
        robotState = RobotState.getInstance();

        elevatorLeader = new SparkMax(Constants.Elevator.leaderID, SparkLowLevel.MotorType.kBrushless);
        elevatorFollower = new SparkMax(Constants.Elevator.followerID, SparkLowLevel.MotorType.kBrushless);

        leaderEncoder = elevatorLeader.getEncoder();
        followerEncoder = elevatorFollower.getEncoder();

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
            .forwardLimitSwitchEnabled(true)
            .reverseLimitSwitchType(Type.kNormallyOpen)
            .reverseLimitSwitchEnabled(true);

        // Soft limits
        elevatorLeaderConfig.softLimit
            .forwardSoftLimit(ElevatorLevel.TOP.getValue())
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(ElevatorLevel.BOTTOM.getValue())
            .reverseSoftLimitEnabled(true);

        double kP = 0.5; // volts/rot 0.5 - 1
        double kI = 0.01; // volts/ rot*s  0.01 - 0.1
        double kD = 0.001;  // volts / rot/s  0.001 - 0.01
        double maxV = 4;

        // These functions can optionally take a slot - e.g. ClosedLoopSlot.kSlot0 is the default
        elevatorLeaderConfig.closedLoop
            .p(kP)
            .i(kI)
            .d(kD)
            .outputRange(-maxV, maxV);

        double kS = 0.2; // volts
        double kG = 2.61; // volts
        double kV = 0.5;  // volts / rot/s
        double kA = 0; // negligible - ignore - 0.032 volts / rot/s

        feedForward = new ElevatorFeedforward(kS, kG, kV, kA);

//        elevatorLeaderConfig.closedLoop.maxMotion
//            .maxVelocity(Constants.Elevator.maxVel)
//            .maxAcceleration(Constants.Elevator.maxAccel)
//            .allowedClosedLoopError(Constants.Elevator.allowedError)

        elevatorLeader.configure(elevatorLeaderConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        elevatorFollowerConfig.apply(globalConfig)
            .follow(elevatorLeader, true);
        elevatorFollower.configure(elevatorFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        controller = elevatorLeader.getClosedLoopController();

        targetLevel = ElevatorLevel.UNKNOWN;
        currentState = ElevatorState.UNKNOWN;
    }

    @Override
    public void periodic() {
        super.periodic();

        double currentPosition = leaderEncoder.getPosition();

        switch (currentState) {
            case HOMING:
                home();
                elevatorLeader.set(elevatorSpeed);
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
                    double ffVoltage = feedForward.calculate(0); // 0 here basically gives us gravity compensation
                    controller.setReference(targetLevel.getValue(),
                        SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVoltage);
                } else {
                    console("Stubbornly refusing to move before homed!", 25);
                }
            default:
                elevatorLeader.set(0);
        }

        if (robotState.getPeriod() != RobotState.StatePeriod.DISABLED) {
            console("position is " + currentPosition + ", target is " + targetLevel.getValue(), 50);
        }

        Logger.recordOutput("Elevator/LeaderEncoder", followerEncoder.getPosition());
        Logger.recordOutput("Elevator/TargetLevel", targetLevel);
    }

    public void setState(ElevatorState state) {
        this.currentState = state;
        // Unless explicitly turned off
        enableSoftLimits(true);

        switch (state) {
            case UNKNOWN:
                hasBeenHomed = false;
                robotState.setElevatorHasBeenHomed(hasBeenHomed);
                break;

            case HOMING:
                enableSoftLimits(false);
                break;

            case HOME:
                console("***** ELEVATOR IS NOW HOME *****");
                hasBeenHomed = true;
                robotState.setElevatorHasBeenHomed(hasBeenHomed);
                leaderEncoder.setPosition(ElevatorLevel.HOME.getValue());
                break;

            case SIMPLE_MOTION:
            case PID_MOTION:
                break;

            case IDLE:
                stopElevator();
                break;
        }
    }

    private void home() {
        setSpeed(0.0);
    }

    public boolean isAtHome() {
        // TODO - the home sequence should be a current limiter if there is no good home switch.
        // but this is pretty safe since the elevator wants to rest at the bottom. So call this 0.
        double encoderPosition = leaderEncoder.getPosition();
        return encoderPosition >= -1 && encoderPosition <= 1;
        // return bottomLimitSwitch.isPressed();
    }

    private void simpleMove(double currentPosition) {
        if (currentPosition < targetLevel.getValue()) {
            setSpeed(Constants.Elevator.simpleUpSpeed);
        } else if (currentPosition > targetLevel.getValue()) {
            setSpeed(-Constants.Elevator.simpleUpSpeed);
        } else {
            setSpeed(0);
        }
    }

    private void setSpeed(double speed) {
        this.elevatorSpeed = Math.abs(speed);
    }

    public ElevatorLevel getTargetLevel() {
        return targetLevel;
    }

    public void setTargetLevel(ElevatorLevel targetLevel) {
        this.targetLevel = targetLevel;
    }

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
        HOME(0),
        BOTTOM(1),
        STORE(Double.NaN),
        LEVEL1(Double.NaN),
        LEVEL2(Double.NaN),
        LEVEL3(15.0),
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
        IDLE,
        SIMPLE_MOTION,
        PID_MOTION,
        HOLD,
    }
}

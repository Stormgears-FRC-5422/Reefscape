// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.onElevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants;
import frc.robot.Constants.SparkConstants;
import frc.robot.RobotState;
import frc.utils.StormSubsystem;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntake extends StormSubsystem {
    private final RobotState robotState;
    private final SparkMax algaeSpark;
    private final SparkMaxConfig algaeConfig;
    private final RelativeEncoder algaeEncoder;

    // Note that this is used to read the value, not directly control the motor
    private double motorSpeed;

    int homeCounter;
    IntakeState currentState;
    private double currentPosition;
    private double targetPosition;
    private boolean hasBeenHomed = false;

    private boolean loaded = false;
    private final SparkClosedLoopController controller;
    private final ArmFeedforward feedForward_loaded;
    private final ArmFeedforward feedForward_unloaded;
    ClosedLoopSlot unloadedSlot = ClosedLoopSlot.kSlot0;
    ClosedLoopSlot loadedSlot = ClosedLoopSlot.kSlot1;

    private final SparkLimitSwitch proximitySensorIntake;

    public AlgaeIntake() {
        robotState = RobotState.getInstance();

        algaeSpark = new SparkMax(Constants.AlgaeIntake.leaderID, SparkLowLevel.MotorType.kBrushless);
        algaeEncoder = algaeSpark.getEncoder();
        proximitySensorIntake = algaeSpark.getForwardLimitSwitch();

        algaeConfig = new SparkMaxConfig();
        algaeConfig.smartCurrentLimit(SparkConstants.Neo550CurrentLimit).idleMode(Constants.AlgaeIntake.brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
        algaeConfig.inverted(Constants.AlgaeIntake.invert);
        algaeConfig.closedLoopRampRate(Constants.AlgaeIntake.closedLoopRampRate);

        double armLength = Constants.AlgaeIntake.armLength;      // meters
        double loadMass = Constants.AlgaeIntake.loadMass;         // kg
        double gravity  = 9.81;        // m/s²

        double kP_loaded = Constants.AlgaeIntake.kP_loaded; // volts/rot 0.5 - 1
        double kI_loaded = Constants.AlgaeIntake.kI_loaded;  // volts/ rot*s  0.01 - 0.1
        double kD_loaded = Constants.AlgaeIntake.kD_loaded;  // volts / rot/s  0.001 - 0.01
        double maxV_loaded = Constants.AlgaeIntake.maxV_loaded;  // volts
        double maxVPct_loaded = maxV_loaded / SparkConstants.NominalVoltage; // percentage (-1 to 1)
//        double maxVPct_loaded = maxV_loaded / 12.0; // percentage (-1 to 1)
        double kS_loaded = Constants.AlgaeIntake.kS_loaded;  // volts
        double kG_loaded = Constants.AlgaeIntake.kG_loaded;  // volts
        double kV_loaded = Constants.AlgaeIntake.kV_loaded;  // volts / rot/s
        double kA_loaded = Constants.AlgaeIntake.kA_loaded;  // negligible - ignore - 0.032 volts / rot/s

        double kP_unloaded = Constants.AlgaeIntake.kP_unloaded; // volts/rot 0.5 - 1
        double kI_unloaded = Constants.AlgaeIntake.kI_unloaded;  // volts/ rot*s  0.01 - 0.1
        double kD_unloaded = Constants.AlgaeIntake.kD_unloaded;  // volts / rot/s  0.001 - 0.01
        double maxV_unloaded = Constants.AlgaeIntake.maxV_unloaded;  // volts
        double maxVPct_unloaded = maxV_unloaded / SparkConstants.NominalVoltage; // percentage (-1 to 1)
        double kS_unloaded = Constants.AlgaeIntake.kS_unloaded;  // volts
        double kG_unloaded = Constants.AlgaeIntake.kG_unloaded;  // volts
        double kV_unloaded = Constants.AlgaeIntake.kV_unloaded;  // volts / rot/s
        double kA_unloaded = Constants.AlgaeIntake.kA_unloaded;  // negligible - ignore - 0.032 volts / rot/s

        algaeConfig.closedLoop
            .p(kP_loaded, loadedSlot)
            .i(kI_loaded, loadedSlot)
            .d(kD_loaded, loadedSlot)
            .outputRange(-maxVPct_loaded, maxVPct_loaded, loadedSlot)
            .p(kP_unloaded, unloadedSlot)
            .i(kI_unloaded, unloadedSlot)
            .d(kD_unloaded, unloadedSlot)
            .outputRange(-maxVPct_unloaded, maxVPct_unloaded, unloadedSlot);

        algaeConfig.closedLoopRampRate(Constants.AlgaeIntake.closedLoopRampRate);
        algaeConfig.openLoopRampRate(Constants.AlgaeIntake.openLoopRampRate);

        algaeSpark.configure(algaeConfig,
            SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        feedForward_loaded = new ArmFeedforward(kS_loaded, kG_loaded, kV_loaded, kA_loaded);
        feedForward_unloaded = new ArmFeedforward(kS_unloaded, kG_unloaded, kV_unloaded, kA_unloaded);

        controller = algaeSpark.getClosedLoopController();
        setState(IntakeState.UNKNOWN);
    }

    @Override
    public void periodic() {
        super.periodic();

        loaded = isLoaded();

        currentPosition = algaeEncoder.getPosition();
        double ffVoltage = 0;
        if (Constants.Debug.debug && robotState.getPeriod() != RobotState.StatePeriod.DISABLED) {
            console("Current position: " + currentPosition + " loaded = " + loaded, 50);
        }

        switch (currentState) {
            case HOMING:
                home();
                algaeSpark.set(motorSpeed);
                break;

            case SIMPLE_MOTION:
                // This only applies when unloaded. This will stall if loaded. Use PID_MOTION
                if (hasBeenHomed) {
                    simpleMove(currentPosition);
                    algaeSpark.set(motorSpeed);
                } else {
                    console("Stubbornly refusing to move before homed!", 25);
                }
                break;

            case PID_MOTION:
                if (hasBeenHomed) {
                    ClosedLoopSlot slot;

                    if (loaded) {
                        // units of velocity are rot/s
                        ffVoltage = feedForward_loaded.calculate(getTheta(), 0); // 0 here basically gives us gravity compensation
                        slot = loadedSlot;
                    } else {
                        ffVoltage = feedForward_unloaded.calculate(getTheta(), 0); // 0 here basically gives us gravity compensation
                        slot = unloadedSlot;
                    }

                    controller.setReference(targetPosition,
                        SparkBase.ControlType.kPosition, slot, ffVoltage);
                } else {
                    console("Stubbornly refusing to move before homed!", 25);
                }
                break;

            default:
                algaeSpark.set(0);
        }

        Logger.recordOutput("AlgaeIntake/AppliedOutput", algaeSpark.getAppliedOutput());
        Logger.recordOutput("AlgaeIntake/Theta", Math.toDegrees(getTheta()));
        Logger.recordOutput("AlgaeIntake/Position", currentPosition);
        Logger.recordOutput("AlgaeIntake/Velocity", algaeEncoder.getVelocity());
        Logger.recordOutput("AlgaeIntake/TargetLevel", targetPosition);
        Logger.recordOutput("AlgaeIntake/Feedforward", ffVoltage);
    }

    public void setState(IntakeState state) {
        currentState = state;

        switch (state) {
            case UNKNOWN -> {
                console("***** UNKNOWN state *****");
                motorSpeed = 0.0;
                targetPosition = IntakeTarget.UNKNOWN.getValue();
            }
            case HOMING -> {
                console("***** HOMING state *****");
                enableSoftLimits(false);
                homeCounter = 0;
            }
            case HOME -> {
                console("***** HOME state *****");
                hasBeenHomed = true;
                robotState.setAlgaeIntakeHasBeenHomed(hasBeenHomed);
                algaeEncoder.setPosition(IntakeTarget.HOME.getValue());
                enableSoftLimits(true);
                motorSpeed = 0.0;
            }
            case PID_MOTION -> {
                console("***** PID_MOTION state *****");

            }
            case SIMPLE_MOTION -> {
                console("***** SIMPLE_MOTION state *****");

            }
        }
    }

    private double getTheta() {
        double radPerRotation = 2.0 * Math.PI / Constants.AlgaeIntake.gearRatio;
        return (currentPosition - Constants.AlgaeIntake.horizontal) * radPerRotation;
    }

    public boolean isAtHome() {
        if (Constants.AlgaeIntake.useCurrentLimitHomeStrategy) {
            // might want to use an average here to minimize spikes
            console("isAtHome output current: " + algaeSpark.getOutputCurrent());
            if (algaeSpark.getOutputCurrent() > Constants.AlgaeIntake.stallCurrentLimit) {
                homeCounter++;
            } else {
                homeCounter = 0;
            }
            return homeCounter > 5;
        } else {
            console("isAtHome current position: " + currentPosition);
            return Math.abs(currentPosition) < Constants.AlgaeIntake.homePositionThreshold;
        }
    }

    private void home() {
        if (Constants.AlgaeIntake.useCurrentLimitHomeStrategy) {
            // might want to use an average here to minimize spikes
            motorSpeed = Constants.AlgaeIntake.stallCurrentSpeed;
        } else {
            motorSpeed = 0.0;
        }
    }

    public boolean isAtTarget() {
        if (loaded) {
            return Math.abs(currentPosition - targetPosition) < Constants.AlgaeIntake.allowedError_loaded;
        } else {
            return Math.abs(currentPosition - targetPosition) < Constants.AlgaeIntake.allowedError_unloaded;
        }
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
        motorSpeed = speed;
    }

    public void setTarget(AlgaeIntake.IntakeTarget target) {
        setTargetPosition(target.getValue());
    }

    public void setTargetPosition(double position) {
        this.targetPosition = position;
    }

    public double getCurrentPosition() {return currentPosition;}

    public void stop() {
        setSpeed(0);
        // This should be redundant, but for safety stop ASAP
        algaeSpark.set(0);
    }

    public boolean isLoaded() {
        // todo - this might need to be more complicated if, say, we get false
        // positives off the CoralIntake, etc.
        return proximitySensorIntake.isPressed();
    }

    public void enableSoftLimits(boolean enable) {
        algaeConfig.softLimit
            .forwardSoftLimit(IntakeTarget.HOME.getValue())
            .forwardSoftLimitEnabled(enable)
            .reverseSoftLimit(IntakeTarget.LOWEST.getValue())
            .reverseSoftLimitEnabled(enable);

        algaeSpark.configure(algaeConfig, SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters);
    }

    public enum IntakeTarget {
        UNKNOWN(Double.NaN),
        HOME(Constants.AlgaeIntake.home),
        STOW(Constants.AlgaeIntake.stow),
        HOLD(Constants.AlgaeIntake.hold),
        HORIZONTAL(Constants.AlgaeIntake.horizontal),
        REEF_PICKUP(Constants.AlgaeIntake.reefPickup),
        GROUND_PICKUP(Constants.AlgaeIntake.groundPickup),
        DROPOFF(Constants.AlgaeIntake.dropoff),
        LOWEST(Constants.AlgaeIntake.lowest);

        private double position;

        IntakeTarget(double position) {
            this.position = position;
        }

        public double getValue() {
            return position;
        }
    }

    public enum IntakeState {
        UNKNOWN,
        HOMING,
        HOME,
        SIMPLE_MOTION,
        PID_MOTION,
    }
}

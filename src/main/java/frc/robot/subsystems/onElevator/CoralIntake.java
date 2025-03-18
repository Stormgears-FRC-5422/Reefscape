// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.onElevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.Constants.SparkConstants;
import frc.robot.RobotState;
import frc.utils.StormSubsystem;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends StormSubsystem {
    private final RobotState robotState;
    private final SparkMax wristSpark;
    private final SparkMaxConfig wristConfig;

    private final SparkMax rollerSpark;
    private final RelativeEncoder wristEncoder;
    // Note that this is used to read the value, not directly control the motor
    private final SparkLimitSwitch proximitySensorIntake;
    private double wristSpeed;
    private double rollerSpeed;

    int homeCounter;
    IntakeState currentState;
    private double currentPosition;
    private boolean hasBeenHomed = false;

    public CoralIntake() {
        wristSpark = new SparkMax(Intake.wristID, SparkLowLevel.MotorType.kBrushless);
        rollerSpark = new SparkMax(Intake.rollerID, SparkLowLevel.MotorType.kBrushless);
        proximitySensorIntake = rollerSpark.getForwardLimitSwitch();

        wristEncoder = wristSpark.getEncoder();
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        wristConfig = new SparkMaxConfig();

        globalConfig.smartCurrentLimit(SparkConstants.Neo550CurrentLimit)
            .voltageCompensation(SparkConstants.Neo550NominalVoltage);

        // No hard limits in this system
        globalConfig.limitSwitch
            .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
            .forwardLimitSwitchEnabled(false)
            .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
            .reverseLimitSwitchEnabled(false);

        // Apply the global config and invert (maybe) according to the config setting
        rollerConfig.apply(globalConfig)
            .inverted(Intake.rollerInvert)
            .idleMode(Intake.rollerBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);

        wristConfig.apply(globalConfig)
            .inverted(Intake.wristInvert)
            .idleMode(Intake.wristBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);

        wristConfig.softLimit
            .forwardSoftLimit(Intake.wristForwardLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(Intake.wristReverseLimit)
            .reverseSoftLimitEnabled(true);

        rollerConfig.softLimit
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false);

        rollerSpark.configure(rollerConfig,
            SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        wristSpark.configure(wristConfig,
            SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        wristConfig.openLoopRampRate(Constants.Intake.wristOpenLoopRampRate);
        rollerConfig.openLoopRampRate(Constants.Intake.rollerOpenLoopRampRate);

        setState(IntakeState.UNKNOWN);
        robotState = RobotState.getInstance();
    }

    @Override
    public void periodic() {
        super.periodic();

        currentPosition = wristEncoder.getPosition();
        double ffVoltage = 0;
//        if (Constants.Debug.debug && robotState.getPeriod() != RobotState.StatePeriod.DISABLED) {
//            console("Current position: " + currentPosition, 50);
//        }

        switch (currentState) {
            case HOMING:
                home();
                wristSpark.set(wristSpeed);
                break;

            case INTAKE:
            case OUTTAKE:
            case GO_HOME:
            case HOLD_UP:
            case READY:
                if (hasBeenHomed) {
                    rollerSpark.set(rollerSpeed);
                    wristSpark.set(wristSpeed);
                } else {
                    console("Stubbornly refusing to move before homed!", 25);
                }
                break;

            case GRIP_CORAL:
                if (hasBeenHomed) {
                    rollerSpark.set(rollerSpeed);
                    wristSpark.set(0);
                } else {
                    console("Stubbornly refusing to move before homed!", 25);
                }
                break;

            case IDLE:
            default:
                rollerSpark.set(0);
                wristSpark.set(0);
        }

        Logger.recordOutput("CoralIntake/WristPosition", currentPosition);

        robotState.setCoralSensorTriggered(isLoaded());
    }

    public void setState(IntakeState state) {
        currentState = state;

        switch (state) {
            case UNKNOWN -> {
                console("***** UNKNOWN state *****");
                rollerSpeed = 0.0;
                wristSpeed = 0.0;
            }
            case HOMING -> {
                console("***** HOMING state *****");
                enableSoftLimits(false);
                homeCounter = 0;
            }
            case HOME -> {
                console("***** HOME state *****");
                hasBeenHomed = true;
                robotState.setIntakeWristHasBeenHomed(hasBeenHomed);
                wristEncoder.setPosition(Constants.Intake.wristHomePosition);
                enableSoftLimits(true);
                rollerSpeed = 0.0;
            }
            case INTAKE -> {
                console("***** INTAKE state *****");
                rollerSpeed = Intake.rollerSpeed;
                wristSpeed = Intake.wristSpeed;
            }
            case OUTTAKE -> {
                console("***** OUTTAKE state *****");
                rollerSpeed = -Intake.rollerSpeed;
                wristSpeed = -Intake.wristSpeed;
            }
            case GO_HOME -> {
                console("***** GO_HOME state *****");
                rollerSpeed = 0;
                wristSpeed = -Intake.wristSpeed;
            }
            case HOLD_UP -> {
                console("***** HOLD_UP state *****");
                rollerSpeed = 0;
                wristSpeed = Intake.wristSpeed;
            }
            case READY -> {
                console("***** READY state *****");
                rollerSpeed = 0;
                wristSpeed = Intake.wristSpeed;
            }
            case IDLE -> {
                rollerSpeed = 0.0;
                wristSpeed = 0.0;
            }
        }
    }

    public boolean isAtHome() {
        if (Constants.Intake.useCurrentLimitHomeStrategy) {
            // might want to use an average here to minimize spikes
            console("isAtHome output current: " + wristSpark.getOutputCurrent());
            if (wristSpark.getOutputCurrent() > Constants.Intake.stallCurrentLimit) {
                homeCounter++;
            } else {
                homeCounter = 0;
            }
            return homeCounter > 5;
        } else {
            console("isAtHome current position: " + currentPosition);
            return Math.abs(currentPosition) < Constants.Intake.homePositionThreshold;
        }
    }

    private void home() {
        if (Constants.Intake.useCurrentLimitHomeStrategy) {
            // might want to use an average here to minimize spikes
            wristSpeed = -Constants.Intake.stallCurrentSpeed;
        } else {
            wristSpeed = 0.0;
        }
    }

    public boolean isLoaded() {
        return proximitySensorIntake.isPressed();
    }

    public void enableSoftLimits(boolean enable) {
        wristConfig.softLimit
            .forwardSoftLimit(IntakePosition.INTAKE.getValue())
            .forwardSoftLimitEnabled(enable)
            .reverseSoftLimit(IntakePosition.OUTTAKE.getValue())
            .reverseSoftLimitEnabled(enable);

        wristSpark.configure(wristConfig, SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters);
    }

    public enum IntakeState {
        UNKNOWN, IDLE, HOMING, HOME, GO_HOME, HOLD_UP, INTAKE, OUTTAKE, GRIP_CORAL, READY;
    }

    public enum IntakePosition {
        UNKNOWN(Double.NaN),
        HOME(Intake.wristHomePosition),
        OUTTAKE(Intake.wristReverseLimit),
        INTAKE(Intake.wristForwardLimit);

        private double position;

        IntakePosition(double position) {
            this.position = position;
        }

        public double getValue() {
            return position;
        }
    }
}


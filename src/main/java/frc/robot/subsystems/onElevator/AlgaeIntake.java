// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.onElevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.Constants.SparkConstants;
import frc.robot.RobotState;
import frc.utils.StormSubsystem;

public class AlgaeIntake extends StormSubsystem {
    private final RobotState robotState;
    private final SparkMax algaeSpark;
    private final SparkMaxConfig algaeConfig;
    private final RelativeEncoder algaeEncoder;

    // Note that this is used to read the value, not directly control the motor
    private double motorSpeed;

    int homeCounter;
    AlgaeIntake.AlgaeIntakeState currentState;
    private double currentPosition;
    private boolean hasBeenHomed = false;

    public AlgaeIntake() {
        algaeSpark = new SparkMax(Constants.AlgaeIntake.leaderID, SparkLowLevel.MotorType.kBrushless);
        algaeEncoder = algaeSpark.getEncoder();

        algaeConfig = new SparkMaxConfig();
        algaeConfig.smartCurrentLimit(SparkConstants.Neo550CurrentLimit).idleMode(IdleMode.kBrake);

        // Apply the global config and invert (maybe) according to the config setting
        algaeConfig.apply(algaeConfig).inverted(Constants.AlgaeIntake.invert);

        algaeSpark.configure(algaeConfig,
            SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        setState(AlgaeIntakeState.UNKNOWN);
        robotState = RobotState.getInstance();
    }

    @Override
    public void periodic() {
        super.periodic();

        currentPosition = algaeEncoder.getPosition();
        double ffVoltage = 0;
        if (Constants.Debug.debug && robotState.getPeriod() != RobotState.StatePeriod.DISABLED) {
            console("Current position: " + currentPosition, 50);
        }

        switch (currentState) {
            case HOMING:
                home();
                algaeSpark.set(motorSpeed);
                break;

            case UP:
            case TRAPPED:
                if (hasBeenHomed) {
                    algaeSpark.set(motorSpeed);
                } else {
                    console("Stubbornly refusing to move before homed!", 25);
                }
                break;

            default:
                algaeSpark.set(0);
        }
    }

    public void setState(AlgaeIntakeState state) {
        currentState = state;

        switch (state) {
            case UNKNOWN -> {
                console("***** UNKNOWN state *****");
                motorSpeed = 0.0;
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
                algaeEncoder.setPosition(Constants.AlgaeIntake.homePosition);
                enableSoftLimits(true);
                motorSpeed = 0.0;
            }
            case UP -> {
                console("***** UP state *****");
                setSpeed(Constants.AlgaeIntake.speed);
            }
            case TRAPPED -> {
                console("***** TRAPPED state *****");
                setSpeed(-Constants.AlgaeIntake.speed);
            }
        }
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
            return Math.abs(currentPosition) < 1.0;
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

    private void setSpeed(double speed) {
        motorSpeed = speed;
    }

    public void enableSoftLimits(boolean enable) {
        algaeConfig.softLimit
            .forwardSoftLimit(Constants.AlgaeIntake.homePosition)
            .forwardSoftLimitEnabled(enable)
            .reverseSoftLimit(Constants.AlgaeIntake.pickupPosition)
            .reverseSoftLimitEnabled(enable);

        algaeSpark.configure(algaeConfig, SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters);
    }

    public enum AlgaePosition {
        MIN(Double.NEGATIVE_INFINITY),
        MAX(Double.POSITIVE_INFINITY);

        private double position;

        AlgaePosition(double position) {
            this.position = position;
        }

        public double getValue() {
            return position;
        }
    }


    public enum AlgaeIntakeState {
        UNKNOWN,
        HOMING,
        HOME,
        UP,
        TRAPPED //spring?

    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.Constants.SparkConstants;
import frc.robot.RobotState;

public class CoralIntake extends SubsystemBase {

    private final RobotState m_robotState;
    private final SparkMax wristSpark;
    private final SparkMax rollerSpark;
    // Note that this is used to read the value, not directly control the motor
    private final SparkLimitSwitch proximitySensorIntake;
    private double wristSpeed;
    private double rollerSpeed;
    public CoralIntake() {
        wristSpark = new SparkMax(Intake.wristID, SparkLowLevel.MotorType.kBrushless);
        rollerSpark = new SparkMax(Intake.rollerID, SparkLowLevel.MotorType.kBrushless);
        proximitySensorIntake = wristSpark.getForwardLimitSwitch();

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        SparkMaxConfig wristConfig = new SparkMaxConfig();

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

        setCoralIntakeState(CoralIntakeState.START);
        m_robotState = RobotState.getInstance();
    }

    @Override
    public void periodic() {
        super.periodic();
        rollerSpark.set(rollerSpeed);
        wristSpark.set(wristSpeed);
        m_robotState.setCoralSensorTriggered(isIntakeSensorTriggered());
    }

    public void setCoralIntakeState(CoralIntakeState state) {
        switch (state) {
            // TODO - do we need a home? Will belt slip?
            case START -> {
                rollerSpeed = 0.0;
                wristSpeed = 0.0;
                wristSpark.getEncoder().setPosition(0.0);
            }
            case OFF -> {
                rollerSpeed = 0.0;
                wristSpeed = 0.0;
            }
            case INTAKE -> {
                rollerSpeed = Intake.rollerSpeed;
                // TODO - need a position pid loop for the wrist.
                wristSpeed = Intake.wristSpeed;
            }
            case OUTTAKE -> {
                rollerSpeed = -Intake.rollerSpeed;
                wristSpeed = 0.0;
            }
        }
    }

    public boolean isIntakeSensorTriggered() {
        return proximitySensorIntake.isPressed();
    }

    public enum CoralIntakeState {
        START, OFF, INTAKE, OUTTAKE;
    }
}


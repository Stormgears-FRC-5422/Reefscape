// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.Constants.SparkConstants;
import frc.robot.RobotState;

public class CoralIntake extends SubsystemBase {
    //different intake states
    public enum CoralIntakeState {
        OFF, INTAKE, OUTTAKE;
    }

    private final SparkMax wristSpark;
    private double wristSpeed;

    private final SparkMax rollerSpark;
    private double rollerSpeed;

    private final DigitalInput proximitySensorIntake;
    private final RobotState m_robotState;

    public CoralIntake() {
        wristSpark = new SparkMax(Intake.wristID, SparkLowLevel.MotorType.kBrushless);
        rollerSpark = new SparkMax(Intake.rollerID, SparkLowLevel.MotorType.kBrushless);
        proximitySensorIntake = new DigitalInput(0);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        SparkMaxConfig wristConfig = new SparkMaxConfig();

        globalConfig.smartCurrentLimit(SparkConstants.Neo550CurrentLimit).idleMode(IdleMode.kBrake);

        globalConfig.limitSwitch
            .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
            .forwardLimitSwitchEnabled(false)
            .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
            .reverseLimitSwitchEnabled(false);

        // Apply the global config and invert (maybe) according to the config setting
        rollerConfig.apply(globalConfig).inverted(Intake.rollerInvert);
        wristConfig.apply(globalConfig).inverted(Intake.wristInvert);

        wristConfig.softLimit
            .forwardSoftLimit(3.0)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(0.0)
            .reverseSoftLimitEnabled(true);

        rollerConfig.softLimit
            .forwardSoftLimit(0.0)
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimit(0.0)
            .reverseSoftLimitEnabled(false);


        // Apply the global config and set to follow the leader
        // The "true" here means invert wrt the leader

        rollerSpark.configure(rollerConfig,
            SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        wristSpark.configure(wristConfig,
            SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        setCoralIntakeState(CoralIntakeState.OFF);
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
            case OFF -> {
                setRollerSpeed(0.0);
                setWristSpeed(0.0);
            }
            case INTAKE -> {
                setRollerSpeed(Intake.rollerSpeed);
                setWristSpeed(Intake.wristSpeed);
            }
            case OUTTAKE -> {
                setRollerSpeed(-Intake.rollerSpeed);
                setWristSpeed(0);
            }
        }
    }

    private void setRollerSpeed(double speed) {
        rollerSpeed = speed;
    }

    private void setWristSpeed(double speed) {
        wristSpeed = speed;
    }

    public boolean isIntakeSensorTriggered(){
        return !proximitySensorIntake.get();
    }

}


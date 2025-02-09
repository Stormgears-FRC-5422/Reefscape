// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SparkConstants;

public class AlgaeIntake extends SubsystemBase {
    //different intake states
    public enum AlgaeIntakeState {
        OFF, INTAKE, OUTTAKE;
    }

    private final SparkMax intakeLeader;

    private double intakeMotorSpeed;

    public AlgaeIntake() {
        intakeLeader = new SparkMax(Constants.AlgaeIntake.leaderID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig intakeLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig intakeFollowerConfig = new SparkMaxConfig();

        globalConfig.smartCurrentLimit(SparkConstants.Neo550CurrentLimit).idleMode(IdleMode.kBrake);

        // Apply the global config and invert (maybe) according to the config setting
        intakeLeaderConfig.apply(globalConfig).inverted(Constants.AlgaeIntake.invertLeader);

        intakeLeader.configure(intakeLeaderConfig,
            SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        setAlgaeIntakeState(AlgaeIntakeState.OFF);
    }

    @Override
    public void periodic() {
        super.periodic();
        intakeLeader.set(intakeMotorSpeed);
    }

    public void setAlgaeIntakeState(AlgaeIntakeState state) {
        switch (state) {
            case OFF -> {
                setSpeed(0.0);
            }
            case INTAKE -> {
                setSpeed(Constants.AlgaeIntake.speed);
            }
            case OUTTAKE -> {
                setSpeed(Constants.AlgaeIntake.speed);
            }
        }
    }

    private void setSpeed(double speed) {
        intakeMotorSpeed = speed;
    }
}

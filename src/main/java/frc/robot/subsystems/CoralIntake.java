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
import frc.robot.Constants.Intake;
import frc.robot.Constants.SparkConstants;

public class CoralIntake extends SubsystemBase {
    //different intake states
    public enum CoralIntakeState {
        OFF, INTAKE, OUTTAKE;
    }

    private final SparkMax intakeLeader;
//    private final SparkMax intakeFollower;
    private double intakeMotorSpeed;

    public CoralIntake() {
        intakeLeader = new SparkMax(Intake.leaderID, SparkLowLevel.MotorType.kBrushless);
//        intakeFollower = new SparkMax(Intake.followerID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig intakeLeaderConfig = new SparkMaxConfig();
//        SparkMaxConfig intakeFollowerConfig = new SparkMaxConfig();

        globalConfig.smartCurrentLimit(SparkConstants.Neo550CurrentLimit).idleMode(IdleMode.kBrake);

        // Apply the global config and invert (maybe) according to the config setting
        intakeLeaderConfig.apply(globalConfig).inverted(Intake.invertLeader);

        // Apply the global config and set to follow the leader
        // The "true" here means invert wrt the leader
//        intakeFollowerConfig.apply(globalConfig).follow(intakeLeader, true);

        intakeLeader.configure(intakeLeaderConfig,
            SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
//        intakeFollower.configure(intakeFollowerConfig,
//            SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        setCoralIntakeState(CoralIntakeState.OFF);
    }

    @Override
    public void periodic() {
        super.periodic();
        intakeLeader.set(intakeMotorSpeed);
    }

    public void setCoralIntakeState(CoralIntakeState state) {
        switch (state) {
            case OFF -> {
                setSpeed(0.0);
            }
            case INTAKE -> {
                setSpeed(Intake.speed);
            }
            case OUTTAKE -> {
                setSpeed(-Intake.speed);
            }
        }
    }

    private void setSpeed(double speed) {
        intakeMotorSpeed = speed;
    }
}

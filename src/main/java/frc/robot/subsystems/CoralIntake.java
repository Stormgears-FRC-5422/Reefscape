// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
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

    private final SparkMax intakeLeader;
    private double intakeMotorSpeed;
    private final DigitalInput proximitySensorIntake;
    private final RobotState m_robotState;

    public CoralIntake() {
        intakeLeader = new SparkMax(Intake.leaderID, SparkLowLevel.MotorType.kBrushless);
        proximitySensorIntake = new DigitalInput(0);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig intakeLeaderConfig = new SparkMaxConfig();

        globalConfig.smartCurrentLimit(SparkConstants.Neo550CurrentLimit).idleMode(IdleMode.kBrake);

        // Apply the global config and invert (maybe) according to the config setting
        intakeLeaderConfig.apply(globalConfig).inverted(Intake.invertLeader);

        // Apply the global config and set to follow the leader
        // The "true" here means invert wrt the leader

        intakeLeader.configure(intakeLeaderConfig,
            SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        setCoralIntakeState(CoralIntakeState.OFF);
        m_robotState = RobotState.getInstance();
    }

    @Override
    public void periodic() {
        super.periodic();
        intakeLeader.set(intakeMotorSpeed);
        m_robotState.setCoralSensorTriggered(isSensorTriggered());
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

    public boolean isSensorTriggered(){
        return !proximitySensorIntake.get();
    }

}


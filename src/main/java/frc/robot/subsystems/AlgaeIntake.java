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

    private final SparkMax intakeLeader;

    private double intakeMotorSpeed;

    public AlgaeIntake() {
        intakeLeader = new SparkMax(Constants.AlgaeIntake.leaderID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig intakeLeaderConfig = new SparkMaxConfig();

        globalConfig.smartCurrentLimit(SparkConstants.Neo550CurrentLimit).idleMode(IdleMode.kBrake);

        // Apply the global config and invert (maybe) according to the config setting
        intakeLeaderConfig.apply(globalConfig).inverted(Constants.AlgaeIntake.invertLeader);

        intakeLeader.configure(intakeLeaderConfig,
            SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        setAlgaeIntakeState(AlgaeIntakeState.TRAPPED);
    }

    @Override
    public void periodic() {
        super.periodic();
        intakeLeader.set(intakeMotorSpeed);
    }

    public void setAlgaeIntakeState(AlgaeIntakeState state) {
        switch (state) {
            case UP -> {
                setSpeed(Constants.AlgaeIntake.speed);
            }
            case TRAPPED -> {
                setSpeed(0.0);
            }
        }
    }
    private void setSpeed(double speed) {
        intakeMotorSpeed = speed;
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
        UP,
        TRAPPED //spring?
    }
}
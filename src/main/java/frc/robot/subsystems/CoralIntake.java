// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.utils.motorcontrol.StormSpark;


public class CoralIntake extends SubsystemBase {
    /**
     * Creates a new Intake.
     */
    public CoralIntake() {
        // private final StormSpark intakeMotor;

        // intakeMotor = new StormSpark(Constants.Intake.intakeID, CANSparkLowLevel.MotorType.kBrushless, StormSpark.MotorKind.k550);
        //intakeMotor.setInverted(true);
        setCoralIntakeState(CoralIntakeState.OFF);
        //set intake motor
    }

    //private final StormSpark intakeMotor;
    //private double intakeMotorSpeed;

    @Override
    public void periodic() {
        super.periodic();
        //intakeMotor.set(intakeMotorSpeed);
    } //speed

    public void setCoralIntakeState(CoralIntakeState state) {
        switch (state) {
            case OFF -> {
                //setSpeed(0.0);
            }
            case FORWARD -> {
                //setSpeed(Constants.Intake.intakeSpeed);
            }
            case REVERSE -> {
                //setSpeed(-Constants.Intake.intakeSpeed);
            }
        }
    }

    private void setSpeed(double speed) {
        //intakeMotorSpeed = speed;
    }
    //different intake states

    public enum CoralIntakeState {
        OFF, FORWARD, REVERSE,
    }
}

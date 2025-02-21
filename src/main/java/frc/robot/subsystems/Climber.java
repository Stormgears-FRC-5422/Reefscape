package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import frc.utils.StormSubsystem;
import frc.utils.motorcontrol.LimitSwitch;

public class Climber extends StormSubsystem {
    public enum ClimberState{
        HOMING,
        HOMED,
        PULLING,
        HOLD,
        ERROR
    }
    ClimberState currentState;
    SparkMax climberMotor;
    LimitSwitch climberLimitSwitch;
    double climberSpeed;

    public Climber() {
        climberMotor = new SparkMax(0, SparkMax.MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        climberSpeed = speed;
    }

    public void home() {
        if (climberLimitSwitch.get()) {
            setSpeed(0);
        } else {
            setSpeed(-0.5);
        }
    }

    public void setState(ClimberState state) {
        currentState = state;
    }
    
    // Fix later
    @Override
    public void periodic() {
        super.periodic();
        switch (currentState) {
            case HOMING:
                home();
                climberMotor.set(climberSpeed);
            case PULLING:
                setSpeed(0.5);
                climberMotor.set(climberSpeed);
            default:
                setSpeed(0);
        }
    }
}

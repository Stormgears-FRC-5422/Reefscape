package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.utils.StormSubsystem;
import org.littletonrobotics.junction.Logger;

public class
Climber extends StormSubsystem {
    final RobotState robotState;

    final SparkMax climberMotor;
    final RelativeEncoder climberEncoder;
    SparkMaxConfig climberMotorConfig;

    ClimberState currentState;
    SparkLimitSwitch climbClosedLimitSwitch;
    SparkLimitSwitch climbOpenLimitSwitch;
    double climberSpeed;

    public Climber() {
        robotState = RobotState.getInstance();

        climberMotor = new SparkMax(Constants.Climber.leaderID, SparkMax.MotorType.kBrushless);

        climberEncoder = climberMotor.getEncoder();

        climberMotorConfig = new SparkMaxConfig();
        SparkMaxConfig globalConfig = new SparkMaxConfig();

        globalConfig.smartCurrentLimit(Constants.Climber.safetyCurrentLimit)
            .voltageCompensation(Constants.Climber.nominalVoltage)
            .idleMode(Constants.Climber.brakeMode ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);

        climberMotorConfig.apply(globalConfig)
            .inverted(Constants.Climber.invertLeader);

        // Hard limits
        climberMotorConfig.limitSwitch
            .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
            .forwardLimitSwitchEnabled(true)
            .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed)
            .reverseLimitSwitchEnabled(true);

        climbClosedLimitSwitch = climberMotor.getForwardLimitSwitch();
        climbOpenLimitSwitch = climberMotor.getReverseLimitSwitch();

        // Soft limits
        climberMotorConfig.softLimit
            .forwardSoftLimit(Constants.Climber.forwardSoftLimit)
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimit(Constants.Climber.reverseSoftLimit)
            .reverseSoftLimitEnabled(false);

        climberMotorConfig.openLoopRampRate(Constants.Climber.openLoopRampRate);

        climberMotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

//        if (Constants.Climber.useFollower) {
//            SparkMax followerMotor = new SparkMax(Constants.Climber.followerID, SparkLowLevel.MotorType.kBrushless);
//            SparkMaxConfig followerMotorConfig = new SparkMaxConfig();
//            followerMotorConfig.apply(globalConfig)
//                .follow(climberMotor, true);
//            followerMotor.configure(followerMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
//        }

        setState(Climber.ClimberState.START);
    }

    public void setSpeed(double speed) {
        climberSpeed = speed;
    }

    public void setState(ClimberState state) {
        currentState = state;

        switch (state) {
            case PULLING -> {
                climberSpeed = Constants.Climber.speed;
            }
            case PUSHING -> {
                climberSpeed = -Constants.Climber.speed;
            }
            case START, IDLE -> {
                climberSpeed = 0.0;
            }

        }
    }

    public boolean isLockedIn() {
        return climbClosedLimitSwitch.isPressed();
    }

    private boolean isFullyForward() {
        return climbOpenLimitSwitch.isPressed();
    }

    @Override
    public void periodic() {
        super.periodic();

        double currentPosition = climberEncoder.getPosition();

        switch (currentState) {
            case PULLING,PUSHING -> {
                climberMotor.set(climberSpeed);
            }
            default -> {
                climberMotor.set(0);
            }
        }

        robotState.setClimberFullyForward(isFullyForward());
        robotState.setClimberLockedIn(isLockedIn());

        Logger.recordOutput("Climber/CurrentPosition", currentPosition);
    }

    public enum ClimberState {
        START,
        PULLING,
        PUSHING,
        HANGING,
        IDLE
    }
}

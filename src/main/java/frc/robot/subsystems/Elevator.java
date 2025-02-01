package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.SparkConstants;
import frc.utils.StormSubsystem;



public class Elevator extends StormSubsystem {

    public enum ElevatorLevel {
        UNKNOWN(Double.NaN),
        FLOOR(Double.NEGATIVE_INFINITY),
        BOTTOM(0),
        STORE(Double.NaN),
        LEVEL1(Double.NaN),
        LEVEL2(Double.NaN),
        LEVEL3(Double.NaN),
        LEVEL4(Double.NaN),
        TOP(20),
        CEILING(Double.POSITIVE_INFINITY);

        private double position;

        ElevatorLevel(double position) {
            this.position = position;
        }

        public double getValue() {
            return position;
        }
    }
    public enum ElevatorState {
        UNKNOWN,
        FLOOR,
        BOTTOM,
        STORE,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        LEVEL4,
        TOP,
        CEILING;
    }

    private double elevatorSpeed;

    private RelativeEncoder m_followerEncoder;
    private RelativeEncoder m_leaderEncoder;

    private SparkLimitSwitch topLimitSwitch;
    private SparkLimitSwitch bottomLimitSwitch;
    

    private final SparkMax elevatorLeader;
    private final SparkMax elevatorFollower;
    private final DigitalInput proximitySensorLevel1;
    private final DigitalInput proximitySensorLevel2;
    private final DigitalInput proximitySensorLevel3;
    private final DigitalInput proximitySensorLevel4;
    private final DigitalInput proximitySensorStore;

    private ElevatorLevel targetLevel;
    private ElevatorState currentState;

    public Elevator() {
        elevatorLeader = new SparkMax(14, SparkLowLevel.MotorType.kBrushless);
        elevatorFollower = new SparkMax(15, SparkLowLevel.MotorType.kBrushless);

        m_leaderEncoder = elevatorLeader.getEncoder();
        m_followerEncoder = elevatorFollower.getEncoder();

        topLimitSwitch = elevatorLeader.getForwardLimitSwitch();
        bottomLimitSwitch = elevatorLeader.getReverseLimitSwitch();

        proximitySensorLevel1 = new DigitalInput(1);
        proximitySensorLevel2 = new DigitalInput(2);
        proximitySensorLevel3 = new DigitalInput(3);
        proximitySensorLevel4 = new DigitalInput(4);
        proximitySensorStore = new DigitalInput(5);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(SparkConstants.CurrentLimit);
        globalConfig.idleMode(IdleMode.kBrake);
        if (Constants.Elevator.brakeMode) {
            globalConfig.idleMode(IdleMode.kBrake);
        } else {
            globalConfig.idleMode(IdleMode.kCoast);
        }
        
        globalConfig.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);

        globalConfig.softLimit
        .forwardSoftLimit(ElevatorLevel.TOP.getValue()) 
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(ElevatorLevel.BOTTOM.getValue()) 
        .reverseSoftLimitEnabled(true);

        SparkMaxConfig elevatorLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();

        elevatorLeaderConfig.apply(globalConfig).inverted(true);
        elevatorFollowerConfig.apply(globalConfig).follow(elevatorLeader, true);

        elevatorLeader.configure(elevatorLeaderConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        elevatorFollower.configure(elevatorFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        targetLevel = ElevatorLevel.UNKNOWN;
        currentState = ElevatorState.UNKNOWN;
        
        
    }

    @Override
    public void periodic() {
        super.periodic();

        double currentPosition = m_leaderEncoder.getPosition();
        
        console("position is " + currentPosition, 50);
        if (elevatorSpeed != 0) {
            if (currentPosition < targetLevel.getValue()) {
                moveElevatorUp();
            } else if (currentPosition > targetLevel.getValue()) {
                moveElevatorDown();
            } else {
                stopElevator();
            }
        } else {
            stopElevator();
        }

        if (topLimitSwitch.isPressed()) {
            stopElevator();
        }
        
        if (bottomLimitSwitch.isPressed()) {
            stopElevator();
        }


        // get position

        // if position < target then
        // move up
        // else move down

        // ifelse position == target
        // stop
        
        switch (targetLevel) {
            case UNKNOWN:

            case FLOOR:

            case BOTTOM:

            case STORE:

            case LEVEL1:
               
            case LEVEL2:
                
            case LEVEL3:
               
            case LEVEL4:

            case TOP:

            case CEILING:
        }
    }

    public void setTargetLevel(ElevatorLevel targetLevel) {
        this.targetLevel = targetLevel;
    }

    public void setSpeed(double speed) {
        this.elevatorSpeed = Math.abs(speed);
    }

    public ElevatorLevel getTargetLevel() {
        return targetLevel;
    }

    private void moveElevatorUp() {
        elevatorLeader.set(elevatorSpeed);
    }

    private void moveElevatorDown() {
        elevatorLeader.set(-elevatorSpeed);
    }

    public void stopElevator() {
        setSpeed(0);
        elevatorLeader.set(0);
    }
}

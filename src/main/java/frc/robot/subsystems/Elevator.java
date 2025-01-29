package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.SparkConstants;
import frc.utils.StormSubsystem;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Elevator extends StormSubsystem {

    public enum ElevatorLevels {
        LEVEL1,
        LEVEL2,
        LEVEL3,
        LEVEL4,
        STORE
    }

    private final SparkMax elevatorLeader;
    private final SparkMax elevatorFollower;
    private final DigitalInput proximitySensorLevel1;
    private final DigitalInput proximitySensorLevel2;
    private final DigitalInput proximitySensorLevel3;
    private final DigitalInput proximitySensorLevel4;
    private final DigitalInput proximitySensorStore;

    private ElevatorLevels elevatorLevel;

    public Elevator() {
        elevatorLeader = new SparkMax(14, SparkLowLevel.MotorType.kBrushless);
        elevatorFollower = new SparkMax(15, SparkLowLevel.MotorType.kBrushless);

        proximitySensorLevel1 = new DigitalInput(1);
        proximitySensorLevel2 = new DigitalInput(2);
        proximitySensorLevel3 = new DigitalInput(3);
        proximitySensorLevel4 = new DigitalInput(4);
        proximitySensorStore = new DigitalInput(5);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(SparkConstants.CurrentLimit);
        globalConfig.idleMode(IdleMode.kBrake);

        SparkMaxConfig elevatorLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();

        elevatorLeaderConfig.apply(globalConfig).inverted(true);
        elevatorFollowerConfig.apply(globalConfig).follow(elevatorLeader, true);

        elevatorLeader.configure(elevatorLeaderConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        elevatorFollower.configure(elevatorFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        elevatorLevel = ElevatorLevels.STORE;
    }

    @Override
    public void periodic() {
        switch (elevatorLevel) {
            case LEVEL1:
                if (!proximitySensorLevel1.get()) {
                    moveElevatorUp();
                } else {
                    stopElevator();
                }
                break;

            case LEVEL2:
                if (!proximitySensorLevel2.get()) {
                    moveElevatorUp();
                } else {
                    stopElevator();
                }
                break;

            case LEVEL3:
                if (!proximitySensorLevel3.get()) {
                    moveElevatorUp();
                } else {
                    stopElevator();
                }
                break;

            case LEVEL4:
                if (!proximitySensorLevel4.get()) {
                    moveElevatorUp();
                } else {
                    stopElevator();
                }
                break;

            case STORE:
                if (!proximitySensorStore.get()) {
                    moveElevatorDown();
                } else {
                    stopElevator();
                }
                break;

            default:
                stopElevator();
                break;
        }
    }

    public void setElevatorLevel(ElevatorLevels state) {
        if (elevatorLevel != ElevatorLevels.STORE) {
            return; //because this code will currently only go up searching for the sensors, it can't go from L2 to L1 so this makes it so it can only go to store if it's on a level
        }
        this.elevatorLevel = state;
    }

    public ElevatorLevels getElevatorLevel() {
        return elevatorLevel;
    }

    public void moveElevatorUp() {
        elevatorLeader.set(0.5);
    }

    public void moveElevatorDown() {
        elevatorLeader.set(-0.5);
    }

    public void stopElevator() {
        elevatorLeader.set(0);
    }
}

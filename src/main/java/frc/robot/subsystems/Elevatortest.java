package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.utils.StormSubsystem;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class Elevatortest extends StormSubsystem {

    private final SparkMax elevatorMotor;
    private final DigitalInput proximitySensor;

    public Elevatortest() {
        elevatorMotor = new SparkMax(99, MotorType.kBrushless);
        proximitySensor = new DigitalInput(1);
    }

    @Override
    public void periodic() {
        if (!proximitySensor.get()) {
            elevatorMotor.set(0.5);
        } else {
            elevatorMotor.set(0);
        }
    }
}

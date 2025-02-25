package frc.robot.subsystems.misc;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pigeon extends SubsystemBase {
    Pigeon2 pigeon;
    public Pigeon() {
        pigeon= new Pigeon2(0, "rio");
        pigeon.setYaw(0);
    }
    public double getYaw() {
        return pigeon.getYaw().getValueAsDouble();
    }
    public Rotation2d getRotation2d() {
        return pigeon.getRotation2d();
    }

    @Override
    public void periodic() {
//        System.out.println(getYaw());
//        RobotState.getInstance().setHeading(getRotation2d());
//        LimelightHelpers.SetRobotOrientation("", -getYaw()-60, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
}

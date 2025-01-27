package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.studica.frc.AHRS;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.utils.vision.LimelightHelpers;

public class NavX extends SubsystemBase {
    private final AHRS navx;
    public Rotation2d getAbsoluteRotation;
    public NavX() {
        navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
        getAbsoluteRotation = Rotation2d.fromDegrees(MathUtil.inputModulus(getYaw(), 180, -180));
        resetHeading();

    }
    public double getYaw() {
        return navx.getYaw();
    }
    public void resetHeading() {
        navx.reset();
    }
    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation("", getYaw(), 0.0, 0.0, 0.0, 0.0, 0.0);
    }
}

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Constants.Drive;
import org.littletonrobotics.junction.Logger;

public class DiagnosticSwerve extends DrivetrainBase {
    TalonFX m_frontLeftDrive, m_frontRightDrive, m_backLeftDrive, m_backRightDrive;
    TalonFX m_frontLeftSteer, m_frontRightSteer, m_backLeftSteer, m_backRightSteer;

    TalonFX[] m_driveArray;
    TalonFX[] m_steerArray;

    public static final double m_maxMotorVoltage = Drive.maxMotorVoltage;

//    PowerDistribution powerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

    public DiagnosticSwerve() {
        // These are convenient lies
        double maxVelocityMetersPerSecond = m_maxMotorVoltage;
        double maxAngularVelocityRadiansPerSecond = m_maxMotorVoltage;

        super.setMaxVelocities(maxVelocityMetersPerSecond * 0.2, maxAngularVelocityRadiansPerSecond * 0.2);

        m_frontLeftDrive = new TalonFX(Drive.frontLeftDriveID);
        m_frontLeftSteer = new TalonFX(Drive.frontLeftSteerID);
        m_frontRightDrive = new TalonFX(Drive.frontRightDriveID);
        m_frontRightSteer = new TalonFX(Drive.frontRightSteerID);

        m_backLeftDrive = new TalonFX(Drive.backLeftDriveID);
        m_backLeftSteer = new TalonFX(Drive.backLeftSteerID);
        m_backRightDrive = new TalonFX(Drive.backRightDriveID);
        m_backRightSteer = new TalonFX(Drive.backRightSteerID);

        m_driveArray = new TalonFX[]{m_frontLeftDrive, m_frontRightDrive,
            m_backLeftDrive, m_backRightDrive};
        m_steerArray = new TalonFX[]{m_frontLeftSteer, m_frontRightSteer,
            m_backLeftSteer, m_backRightSteer};

        // TODO Initialize to known states
//        for (TalonFX m : m_driveArray) {
//            m.setInverted(!m.equals(m_frontLeftDrive) && !m.equals(m_backLeftDrive));
//            m.setIdleMode(CANSparkBase.IdleMode.kCoast);
//            m.getEncoder().setAverageDepth(8);
//            m.getEncoder().setMeasurementPeriod(8);
//            m.setSmartCurrentLimit(80);
//        }

//        for (TalonFX m : m_steerArray) {
//            m.setInverted(true);
//            m.setIdleMode(CANSparkBase.IdleMode.kCoast);
//        }
    }

    @Override
    public void periodic() {
        double driveSpeed = m_chassisSpeeds.vxMetersPerSecond;
        double steerSpeed = m_chassisSpeeds.omegaRadiansPerSecond;

        for (TalonFX m : m_driveArray) {
            m.set(driveSpeed);
        }

        for (TalonFX m : m_steerArray) {
            m.set(steerSpeed);
        }

//        Logger.recordOutput("Module Number 1 " , powerDistribution.getCurrent(13));
//        Logger.recordOutput("Module Number 2 " , powerDistribution.getCurrent(8));
//        Logger.recordOutput("Module Number 3 " , powerDistribution.getCurrent(1));
//        Logger.recordOutput("Module Number 4 " , powerDistribution.getCurrent(18));

        for (TalonFX m : m_driveArray) {
            Logger.recordOutput("Position drive module " + m.getDeviceID(), m.getPosition().getValue());
            Logger.recordOutput("Velocity drive module " + m.getDeviceID(), m.getVelocity().getValue());
// TODO            Logger.recordOutput("Volts drive module " + m.getDeviceID(), m.get  getAppliedOutput() * m.getBusVoltage());
        }
        for (TalonFX m : m_steerArray) {
            Logger.recordOutput("Position steer module " + m.getDeviceID(), m.getPosition().getValue());
            Logger.recordOutput("Velocity steer module " + m.getDeviceID(), m.getVelocity().getValue());
// TODO           Logger.recordOutput("Volts steer module " + m.getDeviceID.(), m.getAppliedOutput() * m.getBusVoltage());
        }
    }
}

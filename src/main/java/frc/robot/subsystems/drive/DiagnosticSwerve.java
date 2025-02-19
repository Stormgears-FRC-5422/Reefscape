package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Constants.Drive;
import frc.robot.Constants.TalonConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

public class DiagnosticSwerve extends DrivetrainBase {
    TalonFX m_frontLeftDrive, m_frontRightDrive, m_backLeftDrive, m_backRightDrive;
    TalonFX m_frontLeftSteer, m_frontRightSteer, m_backLeftSteer, m_backRightSteer;
    CANcoder m_frontLeftEncoder, m_frontRightEncoder, m_backLeftEncoder, m_backRightEncoder;

    TalonFX[] m_driveArray;
    TalonFX[] m_steerArray;
    CANcoder[] m_encoderArray;

    public static final double m_maxMotorVoltage = Drive.maxMotorVoltage;

//    PowerDistribution powerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

    public DiagnosticSwerve() {
        // These are convenient lies
        double maxVelocityMetersPerSecond = m_maxMotorVoltage;
        double maxAngularVelocityRadiansPerSecond = m_maxMotorVoltage;
        setMaxVelocities(maxVelocityMetersPerSecond * Drive.precisionSpeedScale,
            maxAngularVelocityRadiansPerSecond * Drive.precisionSpeedScale);

        String CANBus = Drive.driveCAN;
        m_frontLeftDrive = new TalonFX(Drive.frontLeftDriveID, CANBus);
        m_frontLeftSteer = new TalonFX(Drive.frontLeftSteerID, CANBus);
        m_frontLeftEncoder = new CANcoder(Drive.frontLeftEncoderID, CANBus);

        m_frontRightDrive = new TalonFX(Drive.frontRightDriveID, CANBus);
        m_frontRightSteer = new TalonFX(Drive.frontRightSteerID, CANBus);
        m_frontRightEncoder = new CANcoder(Drive.frontRightEncoderID, CANBus);

        m_backLeftDrive = new TalonFX(Drive.backLeftDriveID, CANBus);
        m_backLeftSteer = new TalonFX(Drive.backLeftSteerID, CANBus);
        m_backLeftEncoder = new CANcoder(Drive.backLeftEncoderID, CANBus);

        m_backRightDrive = new TalonFX(Drive.backRightDriveID, CANBus);
        m_backRightSteer = new TalonFX(Drive.backRightSteerID, CANBus);
        m_backRightEncoder = new CANcoder(Drive.backRightEncoderID, CANBus);

        m_frontLeftSteer.setPosition(0);
        m_frontRightSteer.setPosition(0);
        m_backLeftSteer.setPosition(0);
        m_backRightSteer.setPosition(0);

        m_driveArray = new TalonFX[]{m_frontLeftDrive, m_frontRightDrive,
            m_backLeftDrive, m_backRightDrive};
        m_steerArray = new TalonFX[]{m_frontLeftSteer, m_frontRightSteer,
            m_backLeftSteer, m_backRightSteer};
        m_encoderArray = new CANcoder[]{m_frontLeftEncoder, m_frontRightEncoder,
            m_backLeftEncoder, m_backRightEncoder};

        var driveLimitConfigs = new CurrentLimitsConfigs();
        driveLimitConfigs.StatorCurrentLimit = TalonConstants.driveStatorCurrentLimit;

        driveLimitConfigs.StatorCurrentLimitEnable = true;
        for (TalonFX m : m_driveArray) {
            m.setNeutralMode(NeutralModeValue.Coast);
            var talonFXConfigurator = m.getConfigurator();
            talonFXConfigurator.apply(driveLimitConfigs);
        }

        var steerLimitConfigs = new CurrentLimitsConfigs();
        steerLimitConfigs.StatorCurrentLimit = TalonConstants.steerStatorCurrentLimit;;
        steerLimitConfigs.StatorCurrentLimitEnable = true;
        for (TalonFX m : m_steerArray) {
            m.setNeutralMode(NeutralModeValue.Coast);
            var talonFXConfigurator = m.getConfigurator();
            talonFXConfigurator.apply(steerLimitConfigs);
        }

        for (CANcoder c : m_encoderArray) {
            ;
        }

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

        double temp = m_frontLeftSteer.getPosition().getValue().in(Degrees);
        Logger.recordOutput("fr", temp-m_frontRightSteer.getPosition().getValue().in(Degrees));
        Logger.recordOutput("bl", temp-m_backLeftSteer.getPosition().getValue().in(Degrees));
        Logger.recordOutput("br", temp-m_backRightSteer.getPosition().getValue().in(Degrees));


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
        for (CANcoder c : m_encoderArray) {
            Logger.recordOutput("Position encoder " + c.getDeviceID(), c.getPosition().getValue());
            Logger.recordOutput("Velocity encoder " + c.getDeviceID(), c.getVelocity().getValue());
        }
    }
}

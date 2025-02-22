package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Constants.ButtonBoard;
import frc.robot.Constants.Drive;
import frc.robot.RobotState;
import frc.robot.ShuffleboardConstants;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.utils.StormCommand;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DrivetrainBase.driveFlip;
import static java.util.Objects.isNull;

public class JoyStickDrive extends StormCommand {
    private final DrivetrainBase drivetrain;
    private final BooleanSupplier robotRelativeSupplier;
//    private final BooleanSupplier turboSupplier;
    private final DoubleSupplier turboSupplier;
    private final DoubleSupplier txSupplier;
    private final DoubleSupplier tySupplier;
    private final DoubleSupplier omegaSupplier;

    private RobotState m_state;
    private boolean m_finish = true;
    private boolean m_flipJoystick = false;
    private final SlewRateLimiter xScaleLimiter = new SlewRateLimiter(Drive.linearRateLimiter); //make it into a constant
    private final SlewRateLimiter yScaleLimiter = new SlewRateLimiter(Drive.linearRateLimiter);
    private final SlewRateLimiter omegaScaleLimiter = new SlewRateLimiter(Drive.turnRateLimiter);

    // Note that we want the joystick to mostly work even if there is no drive.
    // So this won't really do anything if the drivetrain is NULL, but it will log.
    public JoyStickDrive(DrivetrainBase drivetrain,
                         ReefscapeJoystick joystick) {
        if (!isNull(drivetrain)) {
            console("adding drivetrain requirements");
            addRequirements(drivetrain);
        } else {
            console("supplied drivetrain is NULL");
        }

        this.drivetrain = drivetrain;

        m_state = RobotState.getInstance();

        txSupplier = joystick::getWpiX;
        tySupplier = joystick::getWpiY;
        omegaSupplier = joystick::getOmegaSpeed;
        robotRelativeSupplier = joystick::getRobotRelative;
        turboSupplier = joystick::getTurbo;

//        ShuffleboardConstants.getInstance().drivetrainTab.add("Drive direction",
//            robotRelativeSupplier.getAsBoolean() ? "Robot Orientation" : "Field Orientation");
    }

    @Override
    public void initialize() {
        super.initialize();

        if (!m_state.isAllianceMissing()) {
            m_flipJoystick = m_state.isAllianceRed() && ButtonBoard.flipJoystickForRed;
            console("Joystick is " + (m_flipJoystick ? "" : "NOT ") + "flipped for alliance");
            m_finish = false;
        } else {
            console("Alliance is not set. Exiting command");
            m_finish = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_finish;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public void execute() {
        if (turboSupplier.getAsDouble() <= 0.2) {
            drivetrain.setDriveSpeedScale(Drive.precisionSpeedScale);
        } else {
            drivetrain.setDriveSpeedScale(turboSupplier.getAsDouble());
        }

        boolean fieldRelative = !robotRelativeSupplier.getAsBoolean();
        double x = txSupplier.getAsDouble();
        Logger.recordOutput("x", x);
        double y = tySupplier.getAsDouble();
        Logger.recordOutput("y", y);
        double omega = omegaSupplier.getAsDouble();
//
        if (Constants.ButtonBoard.squarePath) {
            x = xScaleLimiter.calculate(x*Math.abs(x));
            y = yScaleLimiter.calculate(y*Math.abs(y));
            omega = omegaScaleLimiter.calculate(omega*Math.abs(omega));
        } else {
            x = xScaleLimiter.calculate(x);
            y = yScaleLimiter.calculate(y);
            omega = omegaScaleLimiter.calculate(omega);
        }

        // When on the red alliance, we want to have "forward" mean "move in the -X direction" and so on.
        // But only for field relative driving. Robot relative driving is always the same
        ChassisSpeeds speeds;
        if (m_flipJoystick && fieldRelative && !driveFlip) {
            speeds = new ChassisSpeeds(-x, -y, -omega);
        } else {
            speeds = new ChassisSpeeds(x, y, omega);
        }
        Logger.recordOutput("joy stick speeds", speeds);
        drivetrain.percentOutputDrive(speeds, fieldRelative);
    }
}

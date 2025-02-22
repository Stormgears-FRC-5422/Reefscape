package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Constants.ButtonBoard;
import frc.robot.Constants.Drive;
import frc.robot.RobotState;
import frc.robot.ShuffleboardConstants;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.utils.StormCommand;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class JoyStickDrive extends StormCommand {
    private final CommandSwerveDrivetrain drivetrain;
    private final BooleanSupplier robotRelativeSupplier;
    private final BooleanSupplier turboSupplier;
    private final DoubleSupplier txSupplier;
    private final DoubleSupplier tySupplier;
    private final DoubleSupplier omegaSupplier;

    private RobotState m_state;
    private boolean m_finish = true;
    private boolean m_flipJoystick = false;
    private final SlewRateLimiter xScaleLimiter = new SlewRateLimiter(Drive.linearRateLimiter);
    private final SlewRateLimiter yScaleLimiter = new SlewRateLimiter(Drive.linearRateLimiter);
    private final SlewRateLimiter omegaScaleLimiter = new SlewRateLimiter(Drive.turnRateLimiter);

    public JoyStickDrive(CommandSwerveDrivetrain drivetrain, ReefscapeJoystick joystick) {
        if (drivetrain != null) {
            addRequirements(drivetrain);
        } else {
            System.out.println("Supplied drivetrain is NULL");
        }
        this.drivetrain = drivetrain;
        m_state = RobotState.getInstance();
        txSupplier = joystick::getWpiX;
        tySupplier = joystick::getWpiY;
        omegaSupplier = joystick::getOmegaSpeed;
        robotRelativeSupplier = joystick::getRobotRelative;
        turboSupplier = joystick::getTurbo;

        ShuffleboardConstants.getInstance().drivetrainTab.add("Drive direction",
            robotRelativeSupplier.getAsBoolean() ? "Robot Orientation" : "Field Orientation");
    }

    @Override
    public void initialize() {
        super.initialize();
        if (!m_state.isAllianceMissing()) {
            // Flip joystick inputs on red alliance if the constant is set
            m_flipJoystick = m_state.isAllianceRed() && ButtonBoard.flipJoystickForRed;
            m_finish = false;
            System.out.println("Joystick is " + (m_flipJoystick ? "" : "NOT ") + "flipped for alliance");
        } else {
            System.out.println("Alliance is not set. Exiting command");
            m_finish = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_finish;
    }

    @Override
    public void execute() {
        // Determine if we are driving field-relative (default) or robot-relative
        boolean fieldRelative = !robotRelativeSupplier.getAsBoolean();
        double x = txSupplier.getAsDouble();
        double y = tySupplier.getAsDouble();
        double omega = omegaSupplier.getAsDouble();

        // Apply square path scaling if enabled
        if (Constants.ButtonBoard.squarePath) {
            x = xScaleLimiter.calculate(x * Math.abs(x));
            y = yScaleLimiter.calculate(y * Math.abs(y));
            omega = omegaScaleLimiter.calculate(omega * Math.abs(omega));
        } else {
            x = xScaleLimiter.calculate(x);
            y = yScaleLimiter.calculate(y);
            omega = omegaScaleLimiter.calculate(omega);
        }

        // Optionally flip joystick inputs when in field-relative mode
        if (m_flipJoystick && fieldRelative) {
            x = -x;
            y = -y;
            omega = -omega;
        }

        // Create chassis speeds from the processed inputs
        ChassisSpeeds speeds = new ChassisSpeeds(x, y, omega);

        // Build the SwerveRequest. In this example we use the field-centric helper;
        // if you require robot-relative control, you may need a separate request type.
        SwerveRequest request = new SwerveRequest.ApplyFieldSpeeds().withSpeeds(speeds);
        
        // Use the drivetrain's applyRequest method to send the command
        drivetrain.applyRequest(() -> request);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}

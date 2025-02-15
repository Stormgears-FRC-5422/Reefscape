package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;

// TODO: Code this (waiting for Krish's code)
//  Align with Reef -> Raise Elevator Level -> Set Arm Position -> Outtake Coral
public class AutoReefCommand extends StormCommand {
    int counter = 0;
    ElevatorLevel level;
    FieldConstants.Side side;
    boolean flag = false;
    DrivetrainBase drivetrainBase;
    VisionSubsystem visionSubsystem;
    ReefscapeJoystick reefscapeJoystick;
    Elevator elevator;
    CoralIntake coralIntake;


    public AutoReefCommand(FieldConstants.Side side,
                           DrivetrainBase drivetrainBase, VisionSubsystem visionSubsystem,
                           ReefscapeJoystick reefscapeJoystick,
                           Elevator.ElevatorLevel level,
                           Elevator elevator,
                           CoralIntake coralIntake) {
        this.side = side;
        this.drivetrainBase = drivetrainBase;
        this.visionSubsystem = visionSubsystem;
        this.reefscapeJoystick = reefscapeJoystick;
        this.level = level;
        this.elevator = elevator;
        this.coralIntake = coralIntake;

        addRequirements(visionSubsystem, drivetrainBase, elevator, coralIntake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Commands.sequence(new AutoReef(drivetrainBase,
            visionSubsystem, reefscapeJoystick, side), )
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return flag;
    }
}


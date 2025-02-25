package frc.robot.commands.autos;

import frc.robot.FieldConstants;
import frc.robot.commands.onElevator.ElevatorMoveToHold;
import frc.robot.commands.onElevator.ElevatorMoveToPosition;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.onElevator.Elevator;
import frc.robot.subsystems.onElevator.Elevator.ElevatorLevel;
import frc.robot.subsystems.vision.VisionSubsystem;
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
    ElevatorMoveToPosition elevatorMoveToPosition;
    ElevatorMoveToHold elevatorMoveToHold;
    Elevator elevator;
//    CoralIntake coralIntake;



    public AutoReefCommand(FieldConstants.Side side,
                           DrivetrainBase drivetrainBase, VisionSubsystem visionSubsystem,
                           Elevator.ElevatorLevel level,
                           Elevator elevator
                           ) {
        this.side = side;
        this.drivetrainBase = drivetrainBase;
        this.visionSubsystem = visionSubsystem;
        this.level = level;
        this.elevatorMoveToPosition = new ElevatorMoveToPosition(elevator, level);
        this.elevatorMoveToHold = new ElevatorMoveToHold(elevator);
//        this.coralIntake = new CoralIntake();

        this.elevator = elevator;

        addRequirements(visionSubsystem, drivetrainBase);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
//        Commands.sequence(new AutoReef(drivetrainBase,
//            visionSubsystem, reefscapeJoystick, side),
//            elevatorMoveToPosition,
//            elevatorMoveToHold);
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


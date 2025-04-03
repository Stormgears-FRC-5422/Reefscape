package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.onElevator.AlgaeIntakeMoveToPosition;
import frc.robot.commands.onElevator.CoralIntakeCommand;
import frc.robot.commands.onElevator.ElevatorMoveToHold;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.onElevator.AlgaeIntake;
import frc.robot.subsystems.onElevator.CoralIntake;
import frc.robot.subsystems.onElevator.Elevator;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.utils.StormCommand;

import java.util.function.BooleanSupplier;

public class AutoAlgae {
    CoralIntake coralIntake;
    AlgaeIntake algaeIntake;
    DrivetrainBase drivetrainBase;
    VisionSubsystem visionSubsystem;
    ReefscapeJoystick joystick;
    Elevator elevator;
//    BooleanSupplier level3;

    public AutoAlgae(CoralIntake coralIntake,
                     AlgaeIntake algaeIntake,
                     DrivetrainBase drivetrainBase,
                     VisionSubsystem visionSubsystem,
                     ReefscapeJoystick joystick,
                     Elevator elevator,
                     BooleanSupplier level3) {
        this.coralIntake = coralIntake;
        this.algaeIntake = algaeIntake;
        this.drivetrainBase = drivetrainBase;
        this.visionSubsystem = visionSubsystem;
        this.joystick = joystick;
        this.elevator = elevator;
//        this.level3 = level3;


    }

    public Command autoAlgaeCommand(BooleanSupplier level3) {
        if (level3.getAsBoolean()) {
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new CoralIntakeCommand(coralIntake, true),
                    new SequentialCommandGroup(new WaitCommand(0.2),
                        new AlgaeIntakeMoveToPosition(algaeIntake, -9))),
                new ParallelCommandGroup(
                    new ParallelRaceGroup(
                        new AutoAlgaeReef(drivetrainBase, visionSubsystem, joystick),
                        new ElevatorMoveToHold(elevator, 7.45)
                    )

                ),
                new ParallelRaceGroup(
                    new CoralIntakeCommand(coralIntake, true),
                    new ElevatorMoveToHold(elevator, 12.69),
                    new WaitCommand(0.2)
                        .andThen(new AlgaeIntakeMoveToPosition(algaeIntake, -3))
                )
            );
        }
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new AutoAlgaeReef(drivetrainBase, visionSubsystem, joystick),
                new ElevatorMoveToHold(elevator, 4.2),
                new CoralIntakeCommand(coralIntake, true),
                new SequentialCommandGroup(new WaitCommand(0.2),
                    new AlgaeIntakeMoveToPosition(algaeIntake, -9))
            )

            ,
            new ParallelRaceGroup(
                new CoralIntakeCommand(coralIntake, true),
                new ElevatorMoveToHold(elevator, 7.52),
                new WaitCommand(0.2)
                    .andThen(new AlgaeIntakeMoveToPosition(algaeIntake, -3))
            )
        );
    }

    public Command autoAlgaeCommandAuto() {

        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new AutoAlgaeReef(drivetrainBase, visionSubsystem, joystick),
                    new ParallelRaceGroup(
                    new CoralIntakeCommand(coralIntake, true),
                    new SequentialCommandGroup(new WaitCommand(0.2),
                        new AlgaeIntakeMoveToPosition(algaeIntake, -9)))),
                new ElevatorMoveToHold(elevator, 4.2)

                )

            ,
            new ParallelRaceGroup(
                new CoralIntakeCommand(coralIntake, true),
                new ElevatorMoveToHold(elevator, 7.52),
                new WaitCommand(0.2)
                    .andThen(new AlgaeIntakeMoveToPosition(algaeIntake, -3))
            )
        );
    }
}



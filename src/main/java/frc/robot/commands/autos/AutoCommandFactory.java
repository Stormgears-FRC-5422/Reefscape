package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.commands.onElevator.*;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.onElevator.AlgaeIntake;
import frc.robot.subsystems.onElevator.CoralIntake;
import frc.robot.subsystems.onElevator.Elevator;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.Optional;

public class AutoCommandFactory {
    private DrivetrainBase drivetrainBase;
    private static AutoFactory autoFactory = null;
    private Elevator elevator;
    private CoralIntake coralIntake;
    private VisionSubsystem vis;
    private ReefscapeJoystick joystick;
    private AlgaeIntake algaeIntake;
    static Optional<? extends Trajectory<?>> cachedTrajectory;
    private int autoReefFirstTagId = -1;
    private int autoReefSecondTagId = -1;
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Routine");
    private Command lastChooserCommand = null;


    public AutoCommandFactory(DrivetrainBase drivetrainBase,
                              Elevator elevator,
                              CoralIntake coralIntake,
                              VisionSubsystem visionSubsystem,
                              ReefscapeJoystick joystick,
                              AlgaeIntake algaeIntake) {
        this.joystick = joystick;
        this.vis = visionSubsystem;
        this.drivetrainBase = drivetrainBase;
        this.coralIntake = coralIntake;
        this.elevator = elevator;
        this.algaeIntake = algaeIntake;

    }

    public Command getChooserAutoCommand() {
        return autoChooser.get();
    }

    public Optional<? extends Trajectory<?>> getChooserTrajectory() {
        if (autoChooser.get() == rightOne()) {
            return loadTrajectory("right_one");
        }
        if (autoChooser.get() == middleOne()) {
            return loadTrajectory("middle_one");
        } else if (autoChooser.get() == leftTwo()) {
            return loadTrajectory("far_left");
        } else {
            return Optional.empty();
        }
    }

    private Command middleOne() {
        return Commands.sequence(
            new PrintCommand("middle"),
            new ParallelCommandGroup(new SequentialCommandGroup(
                new AutoReef(drivetrainBase, vis, joystick, () -> FieldConstants.Side.RIGHT)),
                home()),

            new AutoReef(drivetrainBase, vis, joystick, () -> FieldConstants.Side.RIGHT),
            new ConditionalCommand(
                new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL4),
                new PrintCommand("Elevator disabled"),
                () -> Constants.Toggles.useElevator),
            Commands.race(
                new ConditionalCommand(
                    new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL4),
                    new PrintCommand("Elevator disabled"),
                    () -> Constants.Toggles.useElevator),
                new ConditionalCommand(
                    new CoralIntakeCommand(coralIntake, false),
                    new PrintCommand("CoralIntake disabled"),
                    () -> Constants.Toggles.useElevator)
            ),
            new ConditionalCommand(
                new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL1),
                new PrintCommand("Elevator disabled"),
                () -> Constants.Toggles.useElevator)
        );
    }

    private Command rightOne() {
        return
            Commands.sequence(
                new PrintCommand("right"),
                new ParallelCommandGroup(
                    new AutoReef(drivetrainBase, vis, joystick, () -> FieldConstants.Side.RIGHT,
                        autoReefFirstTagId)
                    ,
                    home().andThen(new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL3))),
//                new InstantCommand(()-> drivetrainBase.drive(new ChassisSpeeds(),false)),
                new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL4),
                Commands.race(
                    new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL4),
                    new CoralIntakeCommand(coralIntake, false)),
                new ParallelCommandGroup(
                    new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL1),
                    autoFactory.trajectoryCmd("right_two")
                        .andThen(new InstantCommand(() -> drivetrainBase.drive(new ChassisSpeeds(), false))),
                    new CoralIntakeCommand(coralIntake, true)),
//                autoFactory.trajectoryCmd("far_left_three"),
//                new InstantCommand(() -> drivetrainBase.drive(new ChassisSpeeds(), false)).andThen(new WaitCommand(0.25)),
                new ParallelRaceGroup(
                    new AutoReef(drivetrainBase, vis, joystick, () -> FieldConstants.Side.RIGHT, autoReefSecondTagId),
                    new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL3))
                ,
                new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL4),
                Commands.race(
                    new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL4),
                    new CoralIntakeCommand(coralIntake, false))
                , new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL1)
            );
    }

    private Command leftTwo() {
        return
            Commands.sequence(
                new PrintCommand("far left"),
                new ParallelCommandGroup(
                    new AutoReef(drivetrainBase, vis, joystick, () -> FieldConstants.Side.RIGHT,
                        autoReefFirstTagId)
                    ,
                    home().andThen(new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL3))),
                new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL4),
                Commands.race(
                    new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL4),
                    new CoralIntakeCommand(coralIntake, false)),
                new ParallelCommandGroup(
                    new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL1),
                    autoFactory.trajectoryCmd("far_left_two")
                        .andThen(new InstantCommand(() -> drivetrainBase.drive(new ChassisSpeeds(), false))),
                    new CoralIntakeCommand(coralIntake, true)),
                new ParallelRaceGroup(
                    new AutoReef(drivetrainBase, vis, joystick, () -> FieldConstants.Side.RIGHT, autoReefSecondTagId),
                    new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL3))
                ,
                new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL4),
                Commands.race(
                    new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL4),
                    new CoralIntakeCommand(coralIntake, false))
                , new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL1)
            );
    }

    public static void loadTrajectories() {
        loadTrajectory("middle_one");
        loadTrajectory("right_one");
        loadTrajectory("left_one");
        loadTrajectory("far_left");
        loadTrajectory("far_left_two");
        loadTrajectory("far_left_three");
        loadTrajectory("far_left2");
    }

    public Pose2d getAutoInitialPose() {
        if (getChooserAutoCommand() != lastChooserCommand) {
            lastChooserCommand = getChooserAutoCommand();
            if (getChooserTrajectory().isPresent() &&
                getChooserTrajectory().get().getInitialPose(RobotState.getInstance().isAllianceRed()).isPresent()) {
                return (getChooserTrajectory().get()
                    .getInitialPose(RobotState.getInstance().isAllianceRed()).get());
            }
        }
        return null;
    }

    public void createAutoFactory() {
        autoFactory = new AutoFactory(
            drivetrainBase::getPose,
            drivetrainBase::declarePoseIsNow,
            drivetrainBase::followTrajectory,
            RobotState.createInstance().isAllianceRed(),
            drivetrainBase
        );
        loadTrajectories();
        autoChooser.addDefaultOption("Nothing", null);
        autoChooser.addOption("MiddleOne", middleOne());
        autoChooser.addOption("LeftTwo", leftTwo());
        autoChooser.addOption("RightOne", rightOne());
        if (RobotState.getInstance().isAllianceRed()) {
            autoReefFirstTagId = 9;
            autoReefSecondTagId = 8;
        } else {
            autoReefFirstTagId = 22;
            autoReefSecondTagId = 17;
        }
        System.out.println("Creating Auto Factory");
    }


    public Command home() {
        return new SequentialCommandGroup(
            new PrintCommand("Homing and drive started"),
            new ParallelCommandGroup(
                new ConditionalCommand(
                    new ElevatorHome(elevator),
                    new PrintCommand("Elevator disabled"),
                    () -> Constants.Toggles.useElevator
                ),
//                new ConditionalCommand(
//                    new AlgaeIntakeHome(algaeIntake),
//                    new PrintCommand("AlgaeIntake disabled"),
//                    () -> Constants.Toggles.useAlgaeIntake
//                ),
                new ConditionalCommand(
                    new CoralIntakeHome(coralIntake),
                    new PrintCommand("CoralIntake disabled"),
                    () -> Constants.Toggles.useCoralIntake
                )));
    }


    private static Optional<? extends Trajectory<?>> loadTrajectory(String trajectoryName) {
        return autoFactory.cache().loadTrajectory(trajectoryName);
    }

}

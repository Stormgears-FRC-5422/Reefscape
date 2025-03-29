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

        if (Constants.Auto.side.equalsIgnoreCase("red")) {
            autoReefFirstTagId = 9;
            autoReefSecondTagId = 8;
        } else {
            autoReefFirstTagId = 22;
            autoReefSecondTagId = 17;
        }

        autoChooser.addDefaultOption("Nothing", null);
        autoChooser.addOption("MiddleOne", middleOne());
        autoChooser.addOption("LeftTwo", leftTwo());
        autoChooser.addOption("RightOne", rightOne());

    }

    public Command getAutoCommand(){
        return autoChooser.get();
    }

    public Command middleOne() {
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
//
//    public Command rightOne() {
//        return Commands.sequence(
////            autoFactory.resetOdometry("right_one"),
//            new ParallelCommandGroup(
//                new AutoReef(drivetrainBase, vis, joystick, () -> FieldConstants.Side.RIGHT),
//                home()),
//            new AutoReef(drivetrainBase, vis, joystick, () -> FieldConstants.Side.RIGHT),
//            new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL4),
//            Commands.race(
//                new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL4),
//                new CoralIntakeCommand(coralIntake, false)),
//            new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL1));
//    }

    public Command rightOne() {
        return
            Commands.sequence(
                new PrintCommand("right"),
                new ParallelCommandGroup(
                    new AutoReef(drivetrainBase, vis, joystick, () -> FieldConstants.Side.RIGHT,
                        autoReefFirstTagId)
//                    autoFactory.trajectoryCmd("far_left").andThen(
//                        new PrintCommand("after left trajectory")
//                        )),
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

    public static void loadTrajectories() {
        String cacheName = Constants.Auto.path;

        loadTrajectory("middle_one");
        loadTrajectory("right_one");
        loadTrajectory("left_one");
        loadTrajectory("far_left");
        loadTrajectory("far_left_two");
        loadTrajectory("far_left_three");
        loadTrajectory("far_left2");

        cachedTrajectory = loadTrajectory(cacheName);
    }

    public static Pose2d getAutoInitialPose() {
        if (cachedTrajectory.isPresent() &&
            cachedTrajectory.get().getInitialPose(RobotState.createInstance()
                .isAllianceRed()).isPresent()) {

            return cachedTrajectory.get().getInitialPose(RobotState.createInstance()
                .isAllianceRed()).get();
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
        System.out.println("Creating Auto Factory");
    }


    public Command leftTwo() {
        return
            Commands.sequence(
                new PrintCommand("far left"),
                new ParallelCommandGroup(
                    new AutoReef(drivetrainBase, vis, joystick, () -> FieldConstants.Side.RIGHT,
                        autoReefFirstTagId)
//                    autoFactory.trajectoryCmd("far_left").andThen(
//                        new PrintCommand("after left trajectory")
//                        )),
                    ,
                    home().andThen(new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL3))),
//                new InstantCommand(()-> drivetrainBase.drive(new ChassisSpeeds(),false)),
                new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL4),
                Commands.race(
                    new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL4),
                    new CoralIntakeCommand(coralIntake, false)),
                new ParallelCommandGroup(
                    new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL1),
                    autoFactory.trajectoryCmd("far_left_two")
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
//        if (loadedTrajectories.containsKey(trajectoryName)) {
//            return loadedTrajectories.get(trajectoryName);
//        } else {
//            loadedTrajectories.put(trajectoryName, Choreo.loadTrajectory(trajectoryName));
//        }
//        return loadedTrajectories.get(trajectoryName);

//    }
        return autoFactory.cache().loadTrajectory(trajectoryName);


    }

    public Command choosePath(String path) {
        return switch (path) {
            case "middle_one" -> middleOne();
            case "right_one" -> rightOne();
            case "far_left" -> leftTwo();
            default -> null;
        };

    }

//    class AutoSelector {
//        private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Routine");
//
//
//        public AutoSelector(AutoCommandFactory autoCommandFactory) {
//            this.autoCommandFactory = autoCommandFactory;
//            PositionChooser.addOption("Middle", "middle_one");
//            PositionChooser.addOption("Far Left", "far_left");
//            PositionChooser.addOption("Right", "right_one");
//            ShuffleboardConstants.getInstance().autoSelectionLayout
//                .add("Starting Position?", PositionChooser)
//                .withPosition(0, 0);
//        }
//
//        public Command buildAuto() {
//            ArrayList<Command> fullRoutine = new ArrayList<>();
//            String selectedPosition = PositionChooser.getSelected();
//
////        if (selectedPosition.equals("far_left")){
////            return autoCommandFactory.farLeft();
////        } else if (selectedPosition.equals("middle_one")){
////            return autoCommandFactory.middleOne();
////        }
//            return null;
//        }
//    }
}

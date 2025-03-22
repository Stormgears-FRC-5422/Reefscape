package frc.robot.commands.autos;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.ShuffleboardConstants;
import frc.robot.commands.onElevator.*;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.onElevator.AlgaeIntake;
import frc.robot.subsystems.onElevator.CoralIntake;
import frc.robot.subsystems.onElevator.Elevator;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

public class AutoCommandFactory {
    private final Optional<Trajectory<SwerveSample>> middleOne = Choreo.loadTrajectory("middle_one");
    private final Optional<Trajectory<SwerveSample>> farLeft = Choreo.loadTrajectory("far_left");
    private final Optional<Trajectory<SwerveSample>> leftOne = Choreo.loadTrajectory("left_one");
    private final Optional<Trajectory<SwerveSample>> rightOne = Choreo.loadTrajectory("right_one");
    private DrivetrainBase drivetrainBase;

    private Timer timer;
    private int count = 0;
    private static AutoFactory autoFactory;
    private Elevator elevator;
    private CoralIntake coralIntake;
    private VisionSubsystem vis;
    private ReefscapeJoystick joystick;
    private AlgaeIntake algaeIntake;
    Command middle = null;
    private static boolean resetOdomtery = false;
    AutoReef autoReef;
    AutoReef autoReef2;
    static Optional<? extends Trajectory<?>> cachedTrajectory;
    static HashMap<String, Optional<? extends Trajectory<?>>> loadedTrajectories = new HashMap<>();

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
        timer = new Timer();
//        autoReef = new AutoReef(drivetrainBase, vis, joystick, () -> FieldConstants.Side.RIGHT);
//        autoReef2 = new AutoReef(drivetrainBase, vis, joystick, () -> FieldConstants.Side.RIGHT);
//        Choreo.loadTrajectory("middle_one");
//        Choreo.loadTrajectory("right_one");
//        Choreo.loadTrajectory("far_left");
//        Choreo.loadTrajectory("far_left_two");
//        Choreo.loadTrajectory("far_left_three");

        autoFactory = new AutoFactory(
            drivetrainBase::getPose,
            drivetrainBase::declarePoseIsNow,
            drivetrainBase::followTrajectory,
            RobotState.createInstance().isAllianceRed(),
            drivetrainBase
        );


//
//        visionSubsystem.setGyro(autoFactory.cache().loadTrajectory("far_left")
//            .get().getInitialPose(RobotState.createInstance().isAllianceRed())
//            .get().getRotation().getDegrees()
//        );
//        if (!resetOdomtery) {
//            autoFactory.resetOdometry("far_left");
//            resetOdomtery = true;
//        }

//        middle = autoFactory.trajectoryCmd("middle_one");


//        autoFactory.resetOdometry("middle_one");
//        autoFactory.resetOdometry("far_left");
    }

//    public Command middleOne() {
//        return Commands.sequence(
//            new PrintCommand("middle"),
//            autoFactory.resetOdometry("middle_one"),
//            autoFactory.trajectoryCmd("middle_one"),
//            new AutoReef(drivetrainBase, vis, joystick, () -> FieldConstants.Side.RIGHT),
//            new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL4),
//            Commands.race(
//                new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL4),
//                new CoralIntakeCommand(coralIntake, false)),
//            new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL1)
//
//        );
//
//    }

    public Command middleOne() {
        return Commands.sequence(
            new PrintCommand("middle"),
            new ParallelCommandGroup(new SequentialCommandGroup(
//                middle.andThen(
//                    new PrintCommand("after middle trajectory")
//                )),
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

    public Command rightOne() {
        return Commands.sequence(
            autoFactory.resetOdometry("right_one"),
            autoFactory.trajectoryCmd("right_one"),
            new AutoReef(drivetrainBase, vis, joystick, () -> FieldConstants.Side.RIGHT),
            new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL4),
            Commands.race(
                new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL4),
                new CoralIntakeCommand(coralIntake, false)),
            new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL1));
    }

    public static void loadTrajectories() {
        String cacheName = "middle_one";

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


//    public Command farLeft() {
//        return
//            Commands.sequence(
//                new PrintCommand("far left"),
//                autoFactory.resetOdometry("far_left"),
//                autoFactory.trajectoryCmd("far_left"),
//                new AutoReef(drivetrainBase, vis, joystick, () -> FieldConstants.Side.RIGHT),
//                new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL4),
//                Commands.race(
//                    new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL4),
//                    new CoralIntakeCommand(coralIntake, false)),
//                new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL1)
//                ,
//                autoFactory.trajectoryCmd("far_left_two"),
//                new CoralIntakeCommand(coralIntake, true),
//                new WaitCommand(1),
//                autoFactory.trajectoryCmd("far_left_three"),
//                new AutoReef(drivetrainBase, vis, joystick, () -> FieldConstants.Side.RIGHT),
//                new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL4),
//                Commands.race(
//                    new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL4),
//                    new CoralIntakeCommand(coralIntake, false))
//                , new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL1)
//            );
//    }

    public Command farLeft() {
        return
            Commands.sequence(
                new PrintCommand("far left"),
                new ParallelCommandGroup(autoReef
//                    autoFactory.trajectoryCmd("far_left").andThen(
//                        new PrintCommand("after left trajectory")
//                        )),
                    ,
                    home()),
//                new InstantCommand(()-> drivetrainBase.drive(new ChassisSpeeds(),false)),
                new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL4),
                Commands.race(
                    new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL4),
                    new CoralIntakeCommand(coralIntake, false)),
                new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL1)
                ,
                autoFactory.trajectoryCmd("far_left_two"),
                new InstantCommand(() -> drivetrainBase.drive(new ChassisSpeeds(), false)),
                new CoralIntakeCommand(coralIntake, true),
                autoFactory.trajectoryCmd("far_left_three"),
                new InstantCommand(() -> drivetrainBase.drive(new ChassisSpeeds(), false)).andThen(new WaitCommand(0.25)),
                autoReef2,
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

    class AutoSelector {
        private SendableChooser<String> PositionChooser = new SendableChooser<>();
        private final AutoCommandFactory autoCommandFactory;

        public AutoSelector(AutoCommandFactory autoCommandFactory) {
            this.autoCommandFactory = autoCommandFactory;
            PositionChooser.addOption("Middle", "middle_one");
            PositionChooser.addOption("Far Left", "far_left");
            PositionChooser.addOption("Right", "right_one");
            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Starting Position?", PositionChooser)
                .withPosition(0, 0);
        }

        public Command buildAuto() {
            ArrayList<Command> fullRoutine = new ArrayList<>();
            String selectedPosition = PositionChooser.getSelected();

//        if (selectedPosition.equals("far_left")){
//            return autoCommandFactory.farLeft();
//        } else if (selectedPosition.equals("middle_one")){
//            return autoCommandFactory.middleOne();
//        }
            return null;
        }
    }
}

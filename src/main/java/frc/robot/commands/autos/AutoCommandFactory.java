package frc.robot.commands.autos;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.ShuffleboardConstants;
import frc.robot.commands.onElevator.CoralIntakeCommand;
import frc.robot.commands.onElevator.ElevatorMoveToHold;
import frc.robot.commands.onElevator.ElevatorMoveToPosition;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.onElevator.CoralIntake;
import frc.robot.subsystems.onElevator.Elevator;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.ArrayList;
import java.util.Optional;

public class AutoCommandFactory {
    private final Optional<Trajectory<SwerveSample>> middleOne = Choreo.loadTrajectory("middle_one");
    private final Optional<Trajectory<SwerveSample>> farLeft = Choreo.loadTrajectory("far_left");
    private final Optional<Trajectory<SwerveSample>> leftOne = Choreo.loadTrajectory("left_one");
    private final Optional<Trajectory<SwerveSample>> rightOne = Choreo.loadTrajectory("right_one");
    private DrivetrainBase drivetrainBase;

    private Timer timer;
    private int count = 0;
    private final AutoFactory autoFactory;
    private  Elevator elevator;
    private  CoralIntake coralIntake;
    private  VisionSubsystem vis;
    private  ReefscapeJoystick joystick;



    public AutoCommandFactory(DrivetrainBase drivetrainBase,
                              Elevator elevator,
                              CoralIntake coralIntake,
                              VisionSubsystem visionSubsystem,
                              ReefscapeJoystick joystick) {
        this.joystick = joystick;
        this.vis = visionSubsystem;
        this.drivetrainBase = drivetrainBase;
        this.coralIntake = coralIntake;
        this.elevator = elevator;
        timer = new Timer();


        autoFactory = new AutoFactory(
            drivetrainBase::getPose,
            drivetrainBase::declarePoseIsNow,
            drivetrainBase::followTrajectory,
            RobotState.createInstance().isAllianceRed(),
            drivetrainBase
        );
    }

    public Command middleOne(){
        return Commands.sequence(
            autoFactory.resetOdometry("middle_one"),
            autoFactory.trajectoryCmd("middle_one"),
            new AutoReef(drivetrainBase,vis, joystick,()-> FieldConstants.Side.RIGHT),
            new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL4),
            Commands.race(
                new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL4),
                new CoralIntakeCommand(coralIntake, false)),
            new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL1)

        );

    }
    public Command rightOne(){
        return Commands.sequence(
            autoFactory.resetOdometry("right_one"),
            autoFactory.trajectoryCmd("right_one"),
            new AutoReef(drivetrainBase,vis, joystick,()-> FieldConstants.Side.RIGHT),
            new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL4),
            Commands.race(
                new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL4),
                new CoralIntakeCommand(coralIntake, false)),
            new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL1)

        );
    }



    public Command farLeft(){
        return
            Commands.sequence(
                autoFactory.resetOdometry("far_left"),
                autoFactory.trajectoryCmd("far_left"),
                new AutoReef(drivetrainBase,vis, joystick,()-> FieldConstants.Side.RIGHT),
                new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL4));//,
//                Commands.race(
//                    new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL4),
//                    new CoralIntakeCommand(coralIntake, false)),
//                new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL1)
//                ,
//                autoFactory.trajectoryCmd("far_left_two"),
//                new CoralIntakeCommand(coralIntake,true),
//                new WaitCommand(1),
//                autoFactory.trajectoryCmd("far_left_three"),
//                new AutoReef(drivetrainBase,vis, joystick,()-> FieldConstants.Side.RIGHT),
//                new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL4),
//                Commands.race(
//                    new ElevatorMoveToHold(elevator, Elevator.ElevatorLevel.LEVEL4),
//                    new CoralIntakeCommand(coralIntake, false))
//                ,new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL1)
//                );
    }

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

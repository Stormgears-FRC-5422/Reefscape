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
import frc.robot.RobotState;
import frc.robot.ShuffleboardConstants;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.ElevatorMoveToHold;
import frc.robot.commands.ElevatorMoveToPosition;
import frc.robot.subsystems.drive.DrivetrainBase;

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
    private  AutoReef autoReef;
    private ElevatorMoveToPosition elevatorMoveToPositionL1;
    private ElevatorMoveToPosition elevatorMoveToPositionL4;
    private ElevatorMoveToHold elevatorL4Hold;
    private ElevatorMoveToHold elevatorL1Hold;
    private CoralIntakeCommand intake;
    private CoralIntakeCommand outake;



    public AutoCommandFactory(DrivetrainBase drivetrainBase, AutoReef autoReef,
                              ElevatorMoveToPosition elevatorMoveToPositionL4,
                              ElevatorMoveToPosition elevatorMoveToPositionL1,
                              ElevatorMoveToHold elevatorL4Hold,
                              ElevatorMoveToHold elevatorL1Hold,
                              CoralIntakeCommand intake,
                              CoralIntakeCommand outake) {
        this.drivetrainBase = drivetrainBase;
        timer = new Timer();
        this.autoReef = autoReef;
        this.elevatorMoveToPositionL1 = elevatorMoveToPositionL1;
        this.elevatorMoveToPositionL4 = elevatorMoveToPositionL4;
        this.elevatorL4Hold = elevatorL4Hold;
        this.elevatorL1Hold = elevatorL1Hold;
        this.intake = intake;
        this.outake = outake;

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
            autoFactory.trajectoryCmd("middle_left"),
            autoReef,
            elevatorMoveToPositionL4,
            Commands.race(
                elevatorL4Hold,
                outake),
            elevatorMoveToPositionL1
        );
    }


    public Command farLeft(){
        return
            Commands.sequence(
                autoFactory.resetOdometry("far_left"),
                autoFactory.trajectoryCmd("far_left"),
                autoReef,
                elevatorMoveToPositionL4,
                Commands.race(
                    elevatorL4Hold,
                    outake),
                elevatorMoveToPositionL1,
                autoFactory.trajectoryCmd("far_left_two"),
                intake,
                autoFactory.trajectoryCmd("far_left_three"),
                autoReef,
                elevatorMoveToPositionL4,
                Commands.race(
                    elevatorL4Hold,
                    outake)
                , elevatorMoveToPositionL1);
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

        if (selectedPosition.equals("far_left")){
            return autoCommandFactory.farLeft();
        } else if (selectedPosition.equals("middle_one")){
            return autoCommandFactory.middleOne();
        }
        return null;
    }
}

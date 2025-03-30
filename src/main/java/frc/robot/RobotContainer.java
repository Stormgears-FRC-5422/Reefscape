// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.onElevator.*;
import frc.robot.commands.JoyStickDrive;
import frc.robot.commands.autos.AutoCommandFactory;
import frc.robot.commands.autos.AutoReef;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.joysticks.IllegalJoystickTypeException;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.joysticks.ReefscapeJoystickFactory;
import frc.robot.subsystems.*;
import frc.robot.subsystems.misc.BatteryMonitor;
import frc.robot.subsystems.misc.ColorSensor;
import frc.robot.subsystems.misc.Lights;
import frc.robot.subsystems.misc.Pigeon;
import frc.robot.subsystems.stormnet.StormNetSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.robot.subsystems.onElevator.AlgaeIntake;
import frc.robot.subsystems.onElevator.CoralIntake;
import frc.robot.subsystems.onElevator.Elevator;
import frc.robot.subsystems.onElevator.Elevator.ElevatorLevel;
import frc.robot.Constants.Toggles;
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.subsystems.vision.StormLimelight;

import java.util.Optional;

import static java.util.Objects.isNull;

public class RobotContainer {
    // **********
    // Subsystems
    // **********
    private BatteryMonitor batteryMonitor;
    private DrivetrainBase drivetrain;
    private VisionSubsystem visionSubsystem;
    private Lights lights;
    private Pigeon pigeon;
    private Climber climber;
    private StormNetSubsystem stormNetSubsystem;

    // **********
    // Fields
    // **********
    final RobotState robotState;

    // **********
    // Commands
    // **********
    private CoralIntake coralIntake;
    private AlgaeIntake algaeIntake;
    private Elevator elevator;
    private ColorSensor colorSensor;

    // Joysticks
    ReefscapeJoystick joystick;
    ReefscapeJoystick buttonBoard;

    // Intake  Outtake
    CoralIntakeCommand coralIntakeCommand;
    CoralIntakeHold coralIntakeHoldUp;
    CoralIntakeHold coralIntakeHoldDown;
    CoralIntakeGrip coralIntakeGrip;
    CoralIntakeCommand coralOuttakeCommand;
    AlgaeIntakeMoveToPosition algaeIntakeCommand;
    AlgaeIntakeMoveToPosition algaeOuttakeCommand;


    // Elevator
    ElevatorDiagnostic moveUpElevator;
    ElevatorDiagnostic moveDownElevator;
    ElevatorManual manualElevator;
    ElevatorMoveToHold toLevel1;
    ElevatorMoveToHold toLevel2;
    ElevatorMoveToHold toLevel3;
    ElevatorMoveToHold toLevel4;

    //Limelights
    StormLimelight[] limelights;

    AutoCommandFactory autoCommandFactory;
    private Command lastChooserCommand = null;

    public RobotContainer() throws IllegalDriveTypeException, IllegalJoystickTypeException {
        console("constructor started");
        robotState = RobotState.getInstance();

        if (Constants.Toggles.useBatteryMonitor) {
            batteryMonitor = new BatteryMonitor();
        }

        if (Constants.Toggles.useDrive) {
            console("Crea" +
                "te drive type " + Constants.Drive.driveType);
            drivetrain = DrivetrainFactory.getInstance(Constants.Drive.driveType, Constants.Drive.driveSubtype);
        }

        if (Constants.Toggles.useCoralIntake) {
            coralIntake = new CoralIntake();
            coralIntakeCommand = new CoralIntakeCommand(coralIntake, true);
            coralIntakeHoldUp = new CoralIntakeHold(coralIntake, CoralIntake.IntakePosition.INTAKE);
            coralIntakeHoldDown = new CoralIntakeHold(coralIntake, CoralIntake.IntakePosition.OUTTAKE);
            coralIntakeGrip = new CoralIntakeGrip(coralIntake);
            coralOuttakeCommand = new CoralIntakeCommand(coralIntake, false);
        }

        if (Toggles.useAlgaeIntake) {
            new Trigger(() -> buttonBoard.autoProcessor()).onTrue(coralIntakeHoldUp.andThen(new AlgaeIntakeHome(algaeIntake)));
            new Trigger(() -> buttonBoard.algaeIntake()).whileTrue(new AlgaeIntakeDiagnostic(algaeIntake, true));
            new Trigger(() -> buttonBoard.algaeOuttake()).whileTrue(new AlgaeIntakeDiagnostic(algaeIntake, false));
        }

        if (Constants.Toggles.useLights) {
            lights = new Lights();
        }

        if (Constants.Toggles.useVision && Toggles.useDrive) {
            limelights = CameraConstants.getReefscapeLimelights();
            visionSubsystem = new VisionSubsystem(drivetrain.getPoseEstimator(), limelights);
        }
        if (Constants.Toggles.useColorSensor) {
            colorSensor = new ColorSensor();
        }


        if (Toggles.useElevator) {
            elevator = new Elevator();
            moveDownElevator = new ElevatorDiagnostic(elevator, false);
            moveUpElevator = new ElevatorDiagnostic(elevator, true);
            toLevel1 = new ElevatorMoveToHold(elevator, ElevatorLevel.LEVEL1);
            toLevel2 = new ElevatorMoveToHold(elevator, ElevatorLevel.LEVEL2);
            toLevel3 = new ElevatorMoveToHold(elevator, ElevatorLevel.LEVEL3);
            toLevel4 = new ElevatorMoveToHold(elevator, ElevatorLevel.LEVEL4);
        }

        if (Toggles.useClimber) {
            climber = new Climber();
        }

        if (Toggles.useStormNet) {
            console("Creating StormNet");
            stormNetSubsystem = new StormNetSubsystem();
        } else {
            console("NOT using StormNet");
        }

        autoCommandFactory = new AutoCommandFactory(drivetrain,
            elevator,
            coralIntake,
            visionSubsystem,
            joystick,
            algaeIntake);

        console("constructor ended");
    }

    private void configureBindings() {
        console("Configuring Joystick bindings");

        if (Toggles.useDrive) {
            new Trigger(() -> joystick.cancelAutoReef()).onTrue(new DummyDriveCommand(drivetrain));
        }

        if (Toggles.useDrive) {
            console("configure button bindings");

            new Trigger(() -> joystick.zeroGyro())
                .onTrue(new InstantCommand(() -> drivetrain.resetOrientation()));
        }

        if (Toggles.useClimber) {
            new Trigger(() -> joystick.climb()).whileTrue(new Climb(climber, true));
            new Trigger(() -> joystick.releaseClimb()).whileTrue(new Climb(climber, false));
        }

        if (Toggles.useElevator) {
            if (Constants.Elevator.useDiagnosticElevator) {
                new Trigger(() -> joystick.elevatorTestPid())
                    .and(robotState::elevatorHasBeenHomed)
                    .whileTrue(new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL3));
            }
        }
    }

    public Command getAutonomousCommand() {
        return autoCommandFactory.getChooserAutoCommand();

    }

    public void configJoysticks() throws IllegalJoystickTypeException {
        if (Toggles.useController) {
            console("Making drive joystick!");
            joystick = ReefscapeJoystickFactory.getInstance(Constants.ButtonBoard.driveJoystick,
                Constants.ButtonBoard.driveJoystickPort);

            configureBindings();

            // Note that this might pass a NULL drive if that is disabled. The JoyStick drive
            // will still work in this case, just not move the robot.
            JoyStickDrive driveWithJoystick = new JoyStickDrive(drivetrain, joystick);
            if (!isNull(drivetrain)) {
                drivetrain.setDefaultCommand(driveWithJoystick);
            }
        }


        if (Toggles.useButtonBoard) {
            console("Making Button Board!");
            buttonBoard = ReefscapeJoystickFactory.getInstance(Constants.ButtonBoard.buttonBoard,
                Constants.ButtonBoard.buttonBoardPort1);

            if (!isNull(elevator) && !Constants.Elevator.useDiagnosticElevator) {
                manualElevator = new ElevatorManual(elevator, buttonBoard);
                elevator.setDefaultCommand(manualElevator);
            }
            configureButtonBoardBindings();

        }
    }

    private void configureButtonBoardBindings() {
        console("Configuring button board bindings");

        if (Toggles.useCoralIntake) {
            new Trigger(() -> buttonBoard.coralIntake()).onTrue(coralIntakeCommand
                .andThen(coralIntakeHoldDown));
            new Trigger(() -> buttonBoard.coralOuttake()).onTrue(coralOuttakeCommand);
        }

        if (Toggles.useAlgaeIntake) {
            new Trigger(() -> buttonBoard.algaeIntake()).onTrue(algaeIntakeCommand);
            new Trigger(() -> buttonBoard.algaeOuttake()).onTrue(algaeOuttakeCommand);
            new Trigger(() -> buttonBoard.algaeOuttake()).onTrue(new AlgaeIntakeMoveToPosition(algaeIntake, AlgaeIntake.IntakeTarget.HOLD));
            new Trigger(() -> buttonBoard.autoProcessor()).onTrue(new AlgaeIntakeMoveToPosition(algaeIntake, AlgaeIntake.IntakeTarget.REEF_PICKUP));
        }

        if (Toggles.useElevator) {

            // In manual mode, buttons L1 - L4 only move elevator
            new Trigger(() -> buttonBoard.elevatorLevel1()).onTrue(toLevel1);
            new Trigger(() -> buttonBoard.elevatorLevel2()).onTrue(toLevel2);
            new Trigger(() -> buttonBoard.elevatorLevel3()).onTrue(toLevel3);
            new Trigger(() -> buttonBoard.elevatorLevel4()).onTrue(toLevel4);
            new Trigger(() -> buttonBoard.elevatorUp()).onTrue(manualElevator);
            new Trigger(() -> buttonBoard.elevatorDown()).onTrue(manualElevator);

        }

        // In auto mode, buttons L1 - L4: move to the right/left reef, move elevator to correct level, and Outtake
        if (Toggles.useAutoReef) {
            new Trigger(() -> buttonBoard.autoReef()).onTrue(
                new AutoReef(drivetrain, visionSubsystem, joystick,
                    () -> (buttonBoard.isRightReef() ? FieldConstants.Side.RIGHT : FieldConstants.Side.LEFT)
                )
            );
        }
    }

    public void updateAlliance() {
        RobotState.StateAlliance a = RobotState.StateAlliance.MISSING;
        String defaultAlliance = Constants.Debug.defaultAlliance.toLowerCase();

        if (!Constants.Debug.debug || defaultAlliance.equals("auto")) {
            DriverStation.refreshData();
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

            if (alliance.isPresent()) {
                a = alliance.get() == DriverStation.Alliance.Blue ? RobotState.StateAlliance.BLUE : RobotState.StateAlliance.RED;
            }
        } else {
            a = defaultAlliance.equals("blue") ? RobotState.StateAlliance.BLUE : RobotState.StateAlliance.RED;
        }

        robotState.setAlliance(a);

        if (!RobotState.getInstance().isAllianceMissing()) {
            if (!RobotState.getInstance().didAllianceUpdated() || RobotState.getInstance().didAllianceChange()) {
                onAllianceUpdated();
                RobotState.getInstance().setLastAlliance();
            }
        }
    }


    public void resetInitialPose() {
        Pose2d initialPose;
        if (Constants.Debug.debug && !robotState.isAllianceMissing()) {
            initialPose = new Pose2d(Constants.Debug.initPoseX, Constants.Debug.initPoseY,
                Rotation2d.fromDegrees(Constants.Debug.initPoseDegrees));
            initialPose = ReefscapeField.remapPose(initialPose, robotState.getAlliance());
        } else {
            // We need to be somewhere. AutoChooser should set the pose itself.
            // This reset only really matter for non-match situations.
            initialPose = new Pose2d();

        }
        if (Toggles.useDrive)
            drivetrain.declarePoseIsNow(initialPose);
    }

    public void updateAutoPose() {
        if (autoCommandFactory.getAutoInitialPose() != null) {
            autoCommandFactory.getAutoInitialPose();
        }
    }

    public void onAllianceUpdated() {
        if (autoCommandFactory != null) {
            autoCommandFactory.initialize();
            RobotState.getInstance().setAllianceUpdated(true);
        }
    }

    // Sequence called from teleopInit
    public void autoHome() {
        new SequentialCommandGroup(
            new ConditionalCommand(new ElevatorHome(elevator),
                new PrintCommand("Elevator disabled"),
                () -> Toggles.useElevator),
//            new ConditionalCommand(new AlgaeIntakeHome(algaeIntake),
//                new PrintCommand("AlgaeIntake` disabled"),
//                () -> Toggles.useAlgaeIntake),
            new ConditionalCommand(new CoralIntakeHome(coralIntake),
                new PrintCommand("CoralIntake disabled"),
                () -> Toggles.useCoralIntake)
        ).schedule();
    }

    public Command home() {
        return new SequentialCommandGroup(
            new PrintCommand("Homing and drive started"),
            new ParallelCommandGroup(
                new ConditionalCommand(
                    new ElevatorHome(elevator),
                    new PrintCommand("Elevator disabled"),
                    () -> Toggles.useElevator
                ),
                new ConditionalCommand(
                    new AlgaeIntakeHome(algaeIntake),
                    new PrintCommand("AlgaeIntake disabled"),
                    () -> Toggles.useAlgaeIntake
                ),
                new ConditionalCommand(
                    new CoralIntakeHome(coralIntake),
                    new PrintCommand("CoralIntake disabled"),
                    () -> Toggles.useCoralIntake
                )));
    }


    public void console(String message) {
        System.out.println("RobotContainer : " + message);
    }
}

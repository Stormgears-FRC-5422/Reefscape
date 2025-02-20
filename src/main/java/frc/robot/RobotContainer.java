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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.JoyStickDrive;
import frc.robot.commands.autos.AutoReef;
import frc.robot.commands.autos.AutoReefCommand;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.joysticks.IllegalJoystickTypeException;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.joysticks.ReefscapeJoystickFactory;
import frc.robot.subsystems.*;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.robot.commands.ElevatorMoveToPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.Constants.Toggles;
import frc.robot.subsystems.vision.StormLimelight;

import java.util.Optional;

import static java.util.Objects.isNull;

public class RobotContainer {
    // **********
    // Subsystems
    // **********
    private DrivetrainBase drivetrain;
    private VisionSubsystem visionSubsystem;
    private NavX navX;
    private Lights lights;
    private Pigeon pigeon;
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
    CoralIntakeCommand coralOuttakeCommand;
    AlgaeIntakeCommand algaeIntakeCommand;
    AlgaeIntakeCommand algaeOuttakeCommand;

    // Auto
    AutoStationCommand autoStationCommand;
    AutoProcessorCommand autoProcessorCommand;
    AutoReefCommand autoReefCommand;
    AutoAlgaeReefCommand autoAlgaeReefCommand;

    // Elevator
    ElevatorDiagnostic moveUpElevator;
    ElevatorDiagnostic moveDownElevator;
    ElevatorManual manualElevator;
    ElevatorMoveToPosition toLevel1;
    ElevatorMoveToPosition toLevel2;
    ElevatorMoveToPosition toLevel3;
    ElevatorMoveToPosition toLevel4;

    //Limelights
    StormLimelight[] limelights;
    StormLimelight limelightReef;


    public RobotContainer() throws IllegalDriveTypeException, IllegalJoystickTypeException {
        console("constructor started");
        robotState = RobotState.getInstance();

        if (Constants.Toggles.useDrive) {
            console("Create drive type " + Constants.Drive.driveType);
            drivetrain = DrivetrainFactory.getInstance(Constants.Drive.driveType, Constants.Drive.driveSubtype);
        }

        if (Constants.Toggles.useCoralIntake) {
            coralIntake = new CoralIntake();
            coralIntakeCommand = new CoralIntakeCommand(coralIntake, true);
            coralOuttakeCommand = new CoralIntakeCommand(coralIntake, false);
        }

        if (Constants.Toggles.useAlgaeIntake) {
            algaeIntake = new AlgaeIntake();
            algaeIntakeCommand = new AlgaeIntakeCommand(algaeIntake, true);
            algaeOuttakeCommand = new AlgaeIntakeCommand(algaeIntake, false);
        }

        if (Constants.Toggles.useLights) {
            lights = new Lights();
        }

        if (Constants.Toggles.useVision) {
            limelightReef = new StormLimelight(Constants.Vision.limelightID);
            limelights = new StormLimelight[1];
            limelights[0] = limelightReef;
            visionSubsystem = new VisionSubsystem(limelights);
            //cameraPose = new CameraPose(visionSubsystem);
            //visionSubsystem.setDefaultCommand(cameraPose);
        }
        if (Constants.Toggles.useColorSensor) {
            colorSensor = new ColorSensor();
        }


        if (Toggles.useElevator) {
            elevator = new Elevator();
            moveDownElevator = new ElevatorDiagnostic(elevator, false);
            moveUpElevator = new ElevatorDiagnostic(elevator, true);
            toLevel1 = new ElevatorMoveToPosition(elevator, ElevatorLevel.LEVEL1);
//            toLevel1 = new ElevatorMoveToHold(elevator, ElevatorLevel.LEVEL1);
            toLevel2 = new ElevatorMoveToPosition(elevator, ElevatorLevel.LEVEL2);
            toLevel3 = new ElevatorMoveToPosition(elevator, ElevatorLevel.LEVEL3);
            toLevel4 = new ElevatorMoveToPosition(elevator, ElevatorLevel.LEVEL4);
        }

        if (Toggles.useAutoReef) {
//            autoReefCommand = new AutoReefCommand(ElevatorLevel.LEVEL4, true);
        }

        if (Toggles.useAutoStation) {
            autoStationCommand = new AutoStationCommand();
        }

        if (Toggles.useAutoAlgaeReef) {
            autoAlgaeReefCommand = new AutoAlgaeReefCommand();
        }

        if (Toggles.useAutoProcessor) {
            autoProcessorCommand = new AutoProcessorCommand();
        }

//        if (Toggles.useController) {
//            console("Making drive joystick!");
//            joystick = ReefscapeJoystickFactory.getInstance(Constants.ButtonBoard.driveJoystick,
//                Constants.ButtonBoard.driveJoystickPort);
//
//            configureBindings();
//
//            // Note that this might pass a NULL drive if that is disabled. The JoyStick drive
//            // will still work in this case, just not move the robot.
//            JoyStickDrive driveWithJoystick = new JoyStickDrive(drivetrain, joystick);
//            if (!isNull(drivetrain)) {
//                drivetrain.setDefaultCommand(driveWithJoystick);
//            }
//        }
//
//
//        if (Toggles.useButtonBoard) {
//            console("Making button board!");
//            buttonBoard = ReefscapeJoystickFactory.getInstance(Constants.ButtonBoard.buttonBoard,
//                Constants.ButtonBoard.buttonBoardPort1);
//            configureButtonBoardBindings();
//
//            if (!isNull(elevator) && !Constants.Elevator.useDiagnosticElevator) {
//                manualElevator = new ElevatorManual(elevator, buttonBoard);
//                elevator.setDefaultCommand(manualElevator);
//            }
//        }

        console("constructor ended");
    }

    private void configureBindings() {
//        if (Constants.Toggles.useCoralIntake) {
//            new Trigger(() -> joystick.coralIntake()).onTrue(coralIntakeCommand);
//            new Trigger(() -> joystick.coralOuttake()).onTrue(coralOuttakeCommand);
            new Trigger(() -> joystick.coralIntake()).onTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        new Trigger(() -> joystick.coralOuttake()).onTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            new Trigger(() -> joystick.zeroWheels()).onTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
            new Trigger(() -> joystick.autoReef()).onTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

//        }
        if (Toggles.useDrive && Toggles.useVision) {
//            temporarily left side
//            new Trigger(() -> joystick.autoReef())
//                .onTrue(new AutoReef(drivetrain, visionSubsystem,
//                    joystick, FieldConstants.Side.LEFT)); //B
            console("configure button bindings");

            new Trigger(() -> joystick.zeroGyro())
                .onTrue(new InstantCommand(() -> drivetrain.resetOrientation()));

        }
        // TODO: uncomment after week zero and change the buttons for diagnostic elevator to avoid conflict
        /*
        if (Toggles.useAlgaeIntake){
            new Trigger(()-> joystick.algaeIntake())
                .onTrue(algaeIntakeCommand);

            new Trigger(()-> joystick.algaeOuttake())
                .onTrue(algaeOuttakeCommand);
        }
         */
        if (Toggles.useElevator) {
            if (Constants.Elevator.useDiagnosticElevator) {
                new Trigger(() -> joystick.elevatorDown())
                    .and(robotState::elevatorHasBeenHomed)
                    .whileTrue(new ElevatorDiagnostic(elevator, false));

                new Trigger(() -> joystick.elevatorUp())
                    .and(robotState::elevatorHasBeenHomed)
                    .whileTrue(new ElevatorDiagnostic(elevator, true));

                new Trigger(() -> joystick.elevatorTestPid())
                    .and(robotState::elevatorHasBeenHomed)
                    .whileTrue(new ElevatorMoveToPosition(elevator, Elevator.ElevatorLevel.LEVEL3));
            }
        }
    }

    public Command getAutonomousCommand() {
//        return new SequentialCommandGroup(new SequentialCommandGroup(
//            new ConditionalCommand(new ElevatorHome(elevator),
//                new PrintCommand("Elevator disabled"),
//                () -> Toggles.useElevator),
//            new ConditionalCommand(new CoralIntakeHome(coralIntake),
//                new PrintCommand("CoralIntake disabled"),
//                () -> Toggles.useCoralIntake)
//        ), new AutoCommandFactory(drivetrain),//,
//            new AutoReef(drivetrain, visionSubsystem,
//                joystick, () -> FieldConstants.Side.RIGHT),
//            new ElevatorMoveToPosition(elevator, ElevatorLevel.LEVEL4),
//            Commands.race(new ElevatorMoveToHold(elevator, ElevatorLevel.LEVEL4),
//                new CoralIntakeCommand(coralIntake, false)));
//
        return Commands.print("hi");


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
            console("Making button board!");
            buttonBoard = ReefscapeJoystickFactory.getInstance(Constants.ButtonBoard.buttonBoard,
                Constants.ButtonBoard.buttonBoardPort1);
            configureButtonBoardBindings();

            if (!isNull(elevator) && !Constants.Elevator.useDiagnosticElevator) {
                manualElevator = new ElevatorManual(elevator, buttonBoard);
                elevator.setDefaultCommand(manualElevator);
            }
        }
    }

    private void configureButtonBoardBindings() {
        console("configure button board bindings");

        if (Toggles.useCoralIntake) {
            new Trigger(() -> buttonBoard.coralIntake()).onTrue(coralIntakeCommand);
            new Trigger(() -> buttonBoard.coralOuttake()).onTrue(coralOuttakeCommand);
        }

        if (Toggles.useAlgaeIntake) {
            new Trigger(() -> buttonBoard.algaeIntake()).onTrue(algaeIntakeCommand);
            new Trigger(() -> buttonBoard.algaeOuttake()).onTrue(algaeOuttakeCommand);
        }

        if (Toggles.useElevator) {
            // unnecessary if manual control is the default command
            // manual joystick on button board
            // new Trigger(() -> buttonBoard.elevatorUp()).whileTrue(moveUpElevator);
            // new Trigger(() -> buttonBoard.elevatorDown()).whileTrue(moveDownElevator);

            // In manual mode, buttons L1 - L4 only move elevator
            new Trigger(() -> buttonBoard.elevatorLevel1()).onTrue(toLevel1);
            new Trigger(() -> buttonBoard.elevatorLevel2()).onTrue(toLevel2);
            new Trigger(() -> buttonBoard.elevatorLevel3()).onTrue(toLevel3);
            new Trigger(() -> buttonBoard.elevatorLevel4()).onTrue(toLevel4);

            // In auto mode, buttons L1 - L4: move to the right/left reef, move elevator to correct level, and Outtake
            if (Toggles.useAutoReef) {
//                new Trigger(() -> buttonBoard.elevatorLevel1AutoRight()).whileTrue(
//                    new AutoReefCommand(ElevatorLevel.LEVEL1, true));
//                new Trigger(() -> buttonBoard.elevatorLevel2AutoRight()).whileTrue(
//                    new AutoReefCommand(ElevatorLevel.LEVEL2, true));
//                new Trigger(() -> buttonBoard.elevatorLevel3AutoRight()).whileTrue(
//                    new AutoReefCommand(ElevatorLevel.LEVEL3, true));
//                new Trigger(() -> buttonBoard.elevatorLevel4AutoRight()).whileTrue(
//                    new AutoReefCommand(ElevatorLevel.LEVEL4, true));
//
//                new Trigger(() -> buttonBoard.elevatorLevel1AutoLeft()).whileTrue(
//                    new AutoReefCommand(ElevatorLevel.LEVEL1, false));
//                new Trigger(() -> buttonBoard.elevatorLevel2AutoLeft()).whileTrue(
//                    new AutoReefCommand(ElevatorLevel.LEVEL2, false));
//                new Trigger(() -> buttonBoard.elevatorLevel3AutoLeft()).whileTrue(
//                    new AutoReefCommand(ElevatorLevel.LEVEL3, false));
//                new Trigger(() -> buttonBoard.elevatorLevel4AutoLeft()).whileTrue(
//                    new AutoReefCommand(ElevatorLevel.LEVEL4, false));

                new Trigger(() -> buttonBoard.autoReef()).onTrue(
                    new AutoReef(drivetrain, visionSubsystem, joystick,
                        () -> (joystick.isRightReef() ? FieldConstants.Side.RIGHT : FieldConstants.Side.LEFT)
                    )
                );

            }
        }

        if (Toggles.useAutoStation) {
            new Trigger(() -> buttonBoard.autoStation()).onTrue(autoStationCommand);
        }

//        if (Toggles.useAutoReef) {
//            // quick auto reef command, outtakes to level 4, right reef with a single button press
//            new Trigger(() -> buttonBoard.autoReef()).onTrue(new AutoReef(drivetrain, visionSubsystem,
//                joystick, FieldConstants.Side.LEFT));
//        }

        if (Toggles.useAutoProcessor) {
            new Trigger(() -> buttonBoard.autoProcessor()).onTrue(autoProcessorCommand);
        }

        if (Toggles.useAutoAlgaeReef) {
            new Trigger(() -> buttonBoard.autoAlgaeReef()).onTrue(autoAlgaeReefCommand);
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

    // Sequence called from teleopInit
    public void autoHome() {
        new SequentialCommandGroup(
            new ConditionalCommand(new ElevatorHome(elevator),
                new PrintCommand("Elevator disabled"),
                () -> Toggles.useElevator),
            new ConditionalCommand(new CoralIntakeHome(coralIntake),
                new PrintCommand("CoralIntake disabled"),
                () -> Toggles.useCoralIntake)
        ).schedule();
    }


    public void console(String message) {
        System.out.println("RobotContainer : " + message);
    }
}

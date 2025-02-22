// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.joysticks.IllegalJoystickTypeException;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.joysticks.ReefscapeJoystickFactory;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.robot.subsystems.drive.ctrGenerated.NovakTunerConstants;
import frc.robot.commands.ElevatorMoveToPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorLevel;


import frc.robot.Constants.Toggles;
import frc.robot.Constants.Debug;
import java.util.Optional;

import static java.util.Objects.isNull;

public class RobotContainer {
    // **********
    // Fields
    // **********
    final RobotState robotState;

    // **********
    // Subsystems
    // **********
    private CoralIntake coralIntake;
    private AlgaeIntake algaeIntake;
    private CommandSwerveDrivetrain drivetrain;
    private VisionSubsystem visionSubsystem;
    private Lights lights;
    private Elevator elevator;
    private ColorSensor colorSensor;

    // **********
    // Commands and Control
    // **********

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


    public RobotContainer() throws IllegalDriveTypeException, IllegalJoystickTypeException {
        console("constructor started");
        robotState = RobotState.getInstance();

        if (Constants.Toggles.useDrive) {
            console("Create drive type " + Constants.Drive.driveType);
            drivetrain = NovakTunerConstants.createPHOENIXDrivetrain();
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

        if (Constants.Toggles.useColorSensor) {
            colorSensor = new ColorSensor();
        }

        if (Toggles.useVision){
            visionSubsystem = new VisionSubsystem("limelight");
        }

        if (Toggles.useElevator) {
            elevator = new Elevator();
            moveDownElevator = new ElevatorDiagnostic(elevator, false);
            moveUpElevator = new ElevatorDiagnostic(elevator, true);
            toLevel1 = new ElevatorMoveToPosition(elevator, ElevatorLevel.LEVEL1);
            toLevel2 = new ElevatorMoveToPosition(elevator, ElevatorLevel.LEVEL2);
            toLevel3 = new ElevatorMoveToPosition(elevator, ElevatorLevel.LEVEL3);
            toLevel4 = new ElevatorMoveToPosition(elevator, ElevatorLevel.LEVEL4);
        }

        if (Toggles.useAutoReef) {
            autoReefCommand = new AutoReefCommand(ElevatorLevel.LEVEL4, true);
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

        console("constructor ended");
    }

    private void configureBindings() {
        console("configure button bindings");

        if (Toggles.useDrive) {
            new Trigger(() -> joystick.zeroGyro())
                .onTrue(new InstantCommand(() -> drivetrain.resetGyro()));
        }

        if (Toggles.useCoralIntake){
            new Trigger(()-> joystick.coralIntake()).onTrue(coralIntakeCommand);
            new Trigger(()-> joystick.coralOuttake()).onTrue(coralOuttakeCommand);
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

    private void configureButtonBoardBindings() {
        console("configure button board bindings");

        if (Toggles.useCoralIntake){
            new Trigger(()-> buttonBoard.coralIntake()).onTrue(coralIntakeCommand);
            new Trigger(()-> buttonBoard.coralOuttake()).onTrue(coralOuttakeCommand);
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
            new Trigger(() -> buttonBoard.elevatorLevel1()).whileTrue(toLevel1);
            new Trigger(() -> buttonBoard.elevatorLevel2()).whileTrue(toLevel2);
            new Trigger(() -> buttonBoard.elevatorLevel3()).whileTrue(toLevel3);
            new Trigger(() -> buttonBoard.elevatorLevel4()).whileTrue(toLevel4);

            // In auto mode, buttons L1 - L4: move to the right/left reef, move elevator to correct level, and Outtake
            if (Toggles.useAutoReef) {
                new Trigger(() -> buttonBoard.elevatorLevel1AutoRight()).whileTrue(
                    new AutoReefCommand(ElevatorLevel.LEVEL1, true));
                new Trigger(() -> buttonBoard.elevatorLevel2AutoRight()).whileTrue(
                    new AutoReefCommand(ElevatorLevel.LEVEL2, true));
                new Trigger(() -> buttonBoard.elevatorLevel3AutoRight()).whileTrue(
                    new AutoReefCommand(ElevatorLevel.LEVEL3, true));
                new Trigger(() -> buttonBoard.elevatorLevel4AutoRight()).whileTrue(
                    new AutoReefCommand(ElevatorLevel.LEVEL4, true));

                new Trigger(() -> buttonBoard.elevatorLevel1AutoLeft()).whileTrue(
                    new AutoReefCommand(ElevatorLevel.LEVEL1, false));
                new Trigger(() -> buttonBoard.elevatorLevel2AutoLeft()).whileTrue(
                    new AutoReefCommand(ElevatorLevel.LEVEL2, false));
                new Trigger(() -> buttonBoard.elevatorLevel3AutoLeft()).whileTrue(
                    new AutoReefCommand(ElevatorLevel.LEVEL3, false));
                new Trigger(() -> buttonBoard.elevatorLevel4AutoLeft()).whileTrue(
                    new AutoReefCommand(ElevatorLevel.LEVEL4, false));
            }
        }

        if (Toggles.useAutoStation) {
            new Trigger(() -> buttonBoard.autoStation()).onTrue(autoStationCommand);
        }

        if (Toggles.useAutoReef) {
            // quick auto reef command, outtakes to level 4, right reef with a single button press
            new Trigger(() -> buttonBoard.autoReef()).onTrue(autoReefCommand);
        }

        if (Toggles.useAutoProcessor) {
            new Trigger(() -> buttonBoard.autoProcessor()).onTrue(autoProcessorCommand);
        }

        if (Toggles.useAutoAlgaeReef) {
            new Trigger(() -> buttonBoard.autoAlgaeReef()).onTrue(autoAlgaeReefCommand);
        }
    }

    public void updateAlliance() {
        RobotState.StateAlliance a = RobotState.StateAlliance.MISSING;
        String defaultAlliance = Debug.defaultAlliance.toLowerCase();

        if ( ! Debug.debug || defaultAlliance.equals("auto") ) {
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

        if (Debug.debug && !robotState.isAllianceMissing()) {
            initialPose = new Pose2d(Debug.initPoseX, Debug.initPoseY,
                Rotation2d.fromDegrees(Debug.initPoseDegrees));
            initialPose = ReefscapeField.remapPose(initialPose, robotState.getAlliance());
        } else {
            // We need to be somewhere. AutoChooser should set the pose itself.
            // This reset only really matter for non-match situations.
            initialPose = new Pose2d();
        }
        if (Toggles.useDrive)
            drivetrain.resetPose(initialPose);
    }

    // Sequence called from teleopInit
    public void autoHome() {
        new SequentialCommandGroup(
            new ConditionalCommand(new ElevatorHome(elevator),
                new PrintCommand("Elevator disabled"),
                () ->Toggles.useElevator),
            new ConditionalCommand(new CoralIntakeHome(coralIntake),
                new PrintCommand("CoralIntake disabled"),
                () ->Toggles.useCoralIntake)
        ).schedule();
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            new ConditionalCommand(new ElevatorHome(elevator),
                new PrintCommand("Elevator disabled"),
                () ->Toggles.useElevator),
            new ConditionalCommand(new CoralIntakeHome(coralIntake),
                new PrintCommand("CoralIntake disabled"),
                () ->Toggles.useCoralIntake)
        );
    }

    public void console(String message) {
        System.out.println("RobotContainer : " + message);
    }
}

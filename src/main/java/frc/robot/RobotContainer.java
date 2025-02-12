// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.joysticks.IllegalJoystickTypeException;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.joysticks.ReefscapeJoystickFactory;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.robot.commands.ElevatorPositionCommand;
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
    private DrivetrainBase drivetrain;
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
    ElevatorHome elevatorHome;
    ElevatorDiagnostic moveUpElevator;
    ElevatorDiagnostic moveDownElevator;
    ElevatorPositionCommand toLevel1;
    ElevatorPositionCommand toLevel2;
    ElevatorPositionCommand toLevel3;
    ElevatorPositionCommand toLevel4;


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

        if (Constants.Toggles.useColorSensor) {
            colorSensor = new ColorSensor();
        }

        if (Toggles.useVision){
            visionSubsystem = new VisionSubsystem("limelight");
        }

        if (Toggles.useElevator) {
            elevator = new Elevator();
            elevatorHome = new ElevatorHome(elevator);
            moveDownElevator = new ElevatorDiagnostic(elevator, false);
            moveUpElevator = new ElevatorDiagnostic(elevator, true);
            toLevel1 = new ElevatorPositionCommand(elevator, ElevatorLevel.LEVEL1);
            toLevel2 = new ElevatorPositionCommand(elevator, ElevatorLevel.LEVEL2);
            toLevel3 = new ElevatorPositionCommand(elevator, ElevatorLevel.LEVEL3);
            toLevel4 = new ElevatorPositionCommand(elevator, ElevatorLevel.LEVEL4);
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

        // Note that this might pass a NULL drive if that is disabled. The JoyStick drive
        // will still work in this case, just not move the robot.
        if (Constants.Toggles.useController) {
            console("Making drive joystick!");
            joystick = ReefscapeJoystickFactory.getInstance(Constants.ButtonBoard.driveJoystick,
                Constants.ButtonBoard.driveJoystickPort);
            JoyStickDrive driveWithJoystick = new JoyStickDrive(drivetrain, joystick);
            if (!isNull(drivetrain)) {
                drivetrain.setDefaultCommand(driveWithJoystick);
            }
        }

        if (Toggles.useController) {
            configureBindings();
        }

        if (Toggles.useButtonBoard) {
            console("Making button board!");
            buttonBoard = ReefscapeJoystickFactory.getInstance(Constants.ButtonBoard.buttonBoard,
                Constants.ButtonBoard.buttonBoardPort1);
            configureButtonBoardBindings();
        }

        console("constructor ended");
    }

    private void configureBindings() {
        console("configure button bindings");

        if (Toggles.useDrive) {
            new Trigger(() -> joystick.zeroGyro())
                .onTrue(new InstantCommand(() -> drivetrain.resetOrientation()));
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
           // TODO - we need to NOT move the elevator if it has not been homed
            new Trigger(() -> joystick.homeElevator())
            .onTrue(new ElevatorHome(elevator));

            new Trigger(() -> joystick.elevatorDown())
            .and(robotState::elevatorHasBeenHomed)
            .whileTrue(new ElevatorDiagnostic(elevator, false));

            new Trigger(() -> joystick.elevatorUp())
            .and(robotState::elevatorHasBeenHomed)
            .whileTrue(new ElevatorDiagnostic(elevator, true));

           new Trigger(() -> joystick.elevatorTestPid())
               .and(robotState::elevatorHasBeenHomed)
               .whileTrue(new ElevatorPositionCommand(elevator, Elevator.ElevatorLevel.LEVEL3));

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
            // manual joystick on button board
            new Trigger(() -> buttonBoard.elevatorUp()).whileTrue(moveUpElevator);
            new Trigger(() -> buttonBoard.elevatorDown()).whileTrue(moveDownElevator);

            // In manual mode, buttons L1 - L4 only move elevator
            if (!buttonBoard.isAutoMode()) {
                new Trigger(() -> buttonBoard.elevatorLevel1()).whileTrue(toLevel1);
                new Trigger(() -> buttonBoard.elevatorLevel2()).whileTrue(toLevel2);
                new Trigger(() -> buttonBoard.elevatorLevel3()).whileTrue(toLevel3);
                new Trigger(() -> buttonBoard.elevatorLevel4()).whileTrue(toLevel4);
            }
            // In auto mode, buttons L1 - L4: move to the right/left reef, move elevator to correct level, and Outtake
            else {
                if (Toggles.useAutoReef) {
                    boolean isRightReef = buttonBoard.isRightReef();
                    new Trigger(() -> buttonBoard.elevatorLevel1()).whileTrue(
                        new AutoReefCommand(ElevatorLevel.LEVEL1, isRightReef));
                    new Trigger(() -> buttonBoard.elevatorLevel1()).whileTrue(
                        new AutoReefCommand(ElevatorLevel.LEVEL2, isRightReef));
                    new Trigger(() -> buttonBoard.elevatorLevel1()).whileTrue(
                        new AutoReefCommand(ElevatorLevel.LEVEL3, isRightReef));
                    new Trigger(() -> buttonBoard.elevatorLevel1()).whileTrue(
                        new AutoReefCommand(ElevatorLevel.LEVEL4, isRightReef));
                }
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


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
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
            drivetrain.declarePoseIsNow(initialPose);
    }

    public void console(String message) {
        System.out.println("RobotContainer : " + message);
    }
}

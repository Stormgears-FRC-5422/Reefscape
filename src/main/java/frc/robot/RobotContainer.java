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
    ReefscapeJoystick joystick;
    ReefscapeJoystick buttonBoard;
    CoralIntakeCommand coralIntakeCommand;
    CoralIntakeCommand coralOuttakeCommand;
    AlgaeIntakeCommand algaeIntakeCommand;
    AlgaeIntakeCommand algaeOuttakeCommand;

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

        // Note that this might pass a NULL drive if that is disabled. The JoyStick drive
        // will still work in this case, just not move the robot.
        if (Constants.Toggles.useController) {
            console("Making drive joystick!");
            joystick = ReefscapeJoystickFactory.getInstance(Constants.ButtonBoard.driveJoystick, Constants.ButtonBoard.driveJoystickPort);
            JoyStickDrive driveWithJoystick = new JoyStickDrive(drivetrain, joystick);
            if (!isNull(drivetrain)) {
                drivetrain.setDefaultCommand(driveWithJoystick);
            }
        }

        if (Toggles.useElevator) {
            elevator = new Elevator();
        }

        if (Toggles.useController) {
            configureBindings();
        }

        if (Toggles.useButtonBoard) {
            console("Making button board!");
            buttonBoard = ReefscapeJoystickFactory.getInstance(Constants.ButtonBoard.buttonBoard, Constants.ButtonBoard.buttonBoardPort1);
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

        /*
        if (Toggles.useAlgaeIntake){
            new Trigger(()-> joystick.algaeIntake())
                .onTrue(algaeIntakeCommand);

            new Trigger(()-> joystick.algaeOuttake())
                .onTrue(algaeOuttakeCommand);
        }
         */
       if (Toggles.useElevator) {
            new Trigger(() -> joystick.homeElevator())
            .onTrue(new HomeElevator(elevator));

            new Trigger(() -> joystick.elevatorDown())
            .and(robotState::elevatorHasBeenHomed)
            .whileTrue(new ElevatorDiagnostic(elevator, false));

            new Trigger(() -> joystick.elevatorUp())
            .and(robotState::elevatorHasBeenHomed)
            .whileTrue(new ElevatorDiagnostic(elevator, true));
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

        // TODO: Fix elevator code below to trigger the correct level. We don't have a button on board for store()?
        if (Toggles.useElevator) {
            new Trigger(() -> buttonBoard.elevatorUp()).whileTrue(new ElevatorDiagnostic(elevator, true));
            new Trigger(() -> buttonBoard.elevatorLevel2()).whileTrue(new ElevatorDiagnostic(elevator, true));
            new Trigger(() -> buttonBoard.elevatorLevel3()).whileTrue(new ElevatorDiagnostic(elevator, true));
            new Trigger(() -> buttonBoard.elevatorLevel4()).whileTrue(new ElevatorDiagnostic(elevator, true));
        }

        // TODO: Add the remaining button board triggers
        /*
        if (Toggles.useAutoStation) {
            //new Trigger(() -> buttonBoard.autoStation()).onTrue(autoStationCommand);
        }

        if (Toggles.useAutoReef) {
            // new Trigger(() -> buttonBoard.autoReef()).onTrue(autoReefCommand);
        }

        if (Toggles.useAutoProcessor) {
            // new Trigger(() -> buttonBoard.autoProcessor()).onTrue(autoProcessorCommand);
        }

        if (Toggles.useAutoAlgaeReef) {
            // new Trigger(() -> buttonBoard.autoAlgaeReef()).onTrue(autoAlgaeReef);
        }
        */
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

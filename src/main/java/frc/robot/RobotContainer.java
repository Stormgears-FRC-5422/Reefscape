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
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.ElevatorDiagnostic;
import frc.robot.commands.JoyStickDrive;
import frc.robot.commands.MoveToLevels;
import frc.robot.joysticks.IllegalJoystickTypeException;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.joysticks.ReefscapeJoystickFactory;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.MoveToLevels;

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
    private DrivetrainBase drivetrain;
    private VisionSubsystem visionSubsystem;
    private Lights lights;
    private Elevator elevator;

    // **********
    // Commands and Control
    // **********
    ReefscapeJoystick joystick;
    CoralIntakeCommand coralIntakeCommand;
    CoralIntakeCommand coralOuttakeCommand;

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

        if (Constants.Toggles.useLights) {
            lights = new Lights();
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

        console("constructor ended");
    }

    private void configureBindings() {
        console("configure button bindings");

        if (Toggles.useDrive) {
            new Trigger(() -> joystick.zeroGyro()).onTrue(new InstantCommand(() -> drivetrain.resetOrientation()));
        }

        if (Toggles.useCoralIntake){
            new Trigger(()-> joystick.coralIntake()).onTrue(coralIntakeCommand);
            new Trigger(()-> joystick.coralOuttake()).onTrue(coralOuttakeCommand);
        }

        if (Toggles.useElevator) {
            new Trigger(() -> joystick.elevatorLevel1()).whileTrue(new ElevatorDiagnostic(elevator, true));
            new Trigger(() -> joystick.store()).whileTrue(new ElevatorDiagnostic(elevator, false));
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

        drivetrain.declarePoseIsNow(initialPose);
    }

    public void console(String message) {
        System.out.println("RobotContainer : " + message);
    }
}

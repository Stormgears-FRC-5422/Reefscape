// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.JoyStickDrive;
import frc.robot.commands.autos.AutoReef;
import frc.robot.commands.vision.CameraPose;
import frc.robot.joysticks.IllegalJoystickTypeException;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.joysticks.ReefscapeJoystickFactory;
import frc.robot.subsystems.*;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;
import frc.robot.subsystems.drive.IllegalDriveTypeException;

import frc.robot.Constants.Toggles;

import java.util.Optional;

import static java.util.Objects.isNull;

public class RobotContainer {
    // **********
    // Subsystems
    // **********
    private CoralIntake coralIntake;
    private DrivetrainBase drivetrain;
    private VisionSubsystem visionSubsystem;
    private NavX navX;
    private CameraPose cameraPose;
    private Lights lights;
    private Pigeon pigeon;
    // **********
    // Fields
    // **********
    final RobotState robotState;

    // **********
    // Commands
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

        if (Constants.Toggles.useVision) {
            visionSubsystem = new VisionSubsystem(Constants.Vision.limelightID);
            //cameraPose = new CameraPose(visionSubsystem);
            //visionSubsystem.setDefaultCommand(cameraPose);
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
        if (Constants.Toggles.useNavX) {
            navX = new NavX();
        }
        if (Constants.Toggles.usePigeon) {
            pigeon = new Pigeon();
        }

        configureBindings();
        console("constructor ended");
    }

    private void configureBindings() {
        if (Constants.Toggles.useCoralIntake) {
//            new Trigger(() -> joystick.coralIntake()).onTrue(coralIntakeCommand);
            new Trigger(() -> joystick.coralOuttake()).onTrue(coralOuttakeCommand);
        }
        if (Toggles.useDrive && Toggles.useVision) {
//            temporarily left side
            new Trigger(() -> joystick.autoReef())
                .onTrue(new AutoReef(drivetrain,visionSubsystem,
                    joystick, FieldConstants.Side.LEFT)); //B
        }
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void updateAlliance() {
        DriverStation.refreshData();
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            robotState.setAlliance(alliance.get() == DriverStation.Alliance.Blue ? RobotState.StateAlliance.BLUE : RobotState.StateAlliance.RED);
        } else {
            robotState.setAlliance(RobotState.StateAlliance.MISSING);
        }
    }

    public void console(String message) {
        System.out.println("RobotContainer : " + message);
    }
}

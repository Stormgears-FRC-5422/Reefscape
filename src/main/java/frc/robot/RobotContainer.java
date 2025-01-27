// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.JoyStickDrive;
import frc.robot.commands.vision.CameraPose;
import frc.robot.joysticks.IllegalJoystickTypeException;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.joysticks.ReefscapeJoystickFactory;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;
import frc.robot.subsystems.drive.IllegalDriveTypeException;

import java.util.Optional;

public class RobotContainer {
    // **********
    // Subsystems
    // **********
    private DrivetrainBase drivetrain;
    private VisionSubsystem visionSubsystem;
    private NavX navX;

    // **********
    // Subsystems
    // **********
    private CameraPose cameraPose;

    // **********
    // Fields
    // **********
    final RobotState robotState;

    // **********
    // Control
    // **********
    ReefscapeJoystick joystick;

    public RobotContainer() throws IllegalDriveTypeException, IllegalJoystickTypeException {
        console("constructor started");
        robotState = RobotState.getInstance();

        if (Constants.Toggles.useDrive) {
            console("Create drive type " + Constants.Drive.driveType);
            drivetrain = DrivetrainFactory.getInstance(Constants.Drive.driveType);
        }

        if (Constants.Toggles.useVision) {
            visionSubsystem = new VisionSubsystem(Constants.Vision.limelightID);
            cameraPose = new CameraPose(visionSubsystem);
            visionSubsystem.setDefaultCommand(cameraPose);
        }

        // Note that this might pass a NULL drive if that is disabled. The JoyStick drive
        // will still work in this case, just not move the robot.
        if (Constants.Toggles.useController) {
            console("Making drive joystick!");
            joystick = ReefscapeJoystickFactory.getInstance(Constants.ButtonBoard.driveJoystick, Constants.ButtonBoard.driveJoystickPort);
            JoyStickDrive driveWithJoystick = new JoyStickDrive(drivetrain, joystick);
            drivetrain.setDefaultCommand(driveWithJoystick);
        }
        if (Constants.Toggles.useNavX) {
            navX = new NavX();
        }

        configureBindings();
        console("constructor ended");
    }

    private void configureBindings() {

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

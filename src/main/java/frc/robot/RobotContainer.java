// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.joysticks.IllegalJoystickTypeException;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.robot.Constants.Toggles;

public class RobotContainer {
    VisionSubsystem visionSubsystem;
    public RobotContainer() throws IllegalDriveTypeException, IllegalJoystickTypeException {
        if (Toggles.useVision){
            visionSubsystem = new VisionSubsystem("limelight");
        }

        configureBindings();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

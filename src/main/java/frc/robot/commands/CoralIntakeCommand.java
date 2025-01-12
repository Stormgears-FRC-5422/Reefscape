// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CoralIntake;
import frc.utils.StormCommand;

import frc.robot.Constants.Intake;

public class CoralIntakeCommand extends StormCommand {
    /**
     * Creates a new Intake.
     */
    private final CoralIntake coralIntake;
    private CoralIntake.CoralIntakeState direction;
    private int counter;

    public CoralIntakeCommand(CoralIntake coralIntake, CoralIntake.CoralIntakeState direction) {
        this.coralIntake = coralIntake;
        this.direction = direction;
        addRequirements(coralIntake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
        console("direction = " + direction);

        coralIntake.setCoralIntakeState(direction);
        counter = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        counter++;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        coralIntake.setCoralIntakeState(CoralIntake.CoralIntakeState.OFF);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return counter >= Intake.intakeIterationCount;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CoralIntake;
import frc.utils.StormCommand;

public class CoralIntakeCommand extends StormCommand {
    /**
     * Store the passed-in Intake.
     */
    private final CoralIntake coralIntake;
    private int counter;

    public CoralIntakeCommand(CoralIntake coralIntake /*,Storage storage*/) {
        this.coralIntake = coralIntake;
        //addRequirements(coralIntake, storage);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();

        coralIntake.setCoralIntakeState(CoralIntake.CoralIntakeState.FORWARD);
        counter = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        super.execute();
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
        return counter >= 20;
    }
}

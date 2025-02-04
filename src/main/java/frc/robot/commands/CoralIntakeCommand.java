// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.Intake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import frc.utils.StormCommand;

public class CoralIntakeCommand extends StormCommand {
    /**
     * Creates a new Intake.
     */
    private final CoralIntake coralIntake;
    private final CoralIntake.CoralIntakeState direction;
    private int counter;
    private int finished_counter;

    public CoralIntakeCommand(CoralIntake coralIntake, boolean intake) {
        this.coralIntake = coralIntake;
        this.direction = intake ? CoralIntakeState.INTAKE : CoralIntakeState.OUTTAKE;
        addRequirements(coralIntake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
        console("direction = " + (direction == CoralIntakeState.INTAKE ? "Intake" : "Outtake"));

        counter = 0;
        finished_counter = 0;
        coralIntake.setCoralIntakeState(direction);
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
        if(coralIntake.isSensorTriggered()) {
            finished_counter++;
        }
        return (counter >= Intake.intakeIterationCount || finished_counter == 5);
    }
}

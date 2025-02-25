// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.onElevator;

import frc.robot.Constants;
import frc.robot.subsystems.onElevator.AlgaeIntake;
import frc.robot.subsystems.onElevator.AlgaeIntake.AlgaeIntakeState;
import frc.utils.StormCommand;


public class AlgaeIntakeCommand extends StormCommand {
    /**
     * Creates a new Intake.
     */
    private final AlgaeIntake algaeIntake;
    private final AlgaeIntakeState direction;
    private int counter;

    public AlgaeIntakeCommand(AlgaeIntake algaeIntake, boolean intake) {
        this.algaeIntake = algaeIntake;
        this.direction = intake ? AlgaeIntakeState.UP : AlgaeIntakeState.TRAPPED;
        addRequirements(algaeIntake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
        console("direction = " + (direction == AlgaeIntakeState.UP ? "Intake" : "Outtake"));

        counter = 0;
        algaeIntake.setState(direction);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        counter++;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return counter >= Constants.AlgaeIntake.intakeIterationCount;
    }
}

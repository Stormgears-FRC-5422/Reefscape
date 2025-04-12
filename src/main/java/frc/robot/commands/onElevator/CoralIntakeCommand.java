// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.onElevator;

import frc.robot.Constants.Intake;
import frc.robot.subsystems.onElevator.CoralIntake;
import frc.robot.subsystems.onElevator.CoralIntake.IntakeState;
import frc.utils.StormCommand;
import edu.wpi.first.wpilibj.Timer;

public class CoralIntakeCommand extends StormCommand {
    private boolean skip;
    /**
     * Creates a new Intake.
     */
    private final CoralIntake coralIntake;
    private final IntakeState operation;
    private int finished_counter;
    private final Timer timer;

    public CoralIntakeCommand(CoralIntake coralIntake, boolean intake) {
        this.coralIntake = coralIntake;
        this.operation = intake ? IntakeState.INTAKE : IntakeState.OUTTAKE;
        timer = new Timer();
        skip = safeAddRequirements(coralIntake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
        if (skip){
            return;
        }
        console("operation = " + (operation == IntakeState.INTAKE ? "Intake" : "Outtake"));

        timer.restart();
        finished_counter = 0;
        coralIntake.setState(operation);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        if(skip) {
            return skip;
        }
        if (operation == IntakeState.INTAKE) {
            // let the motor run for a few iterations after sensor is triggered to fully align Coral with the base
            if (coralIntake.isLoaded()) {
                finished_counter++;
            } else {
                finished_counter = 0;
            }
            return (timer.get() >= Intake.rollerIntakeDuration || finished_counter == 5);
        } else {
            return (timer.get() >= Intake.rollerOuttakeDuration);
        }
//        if (operation == IntakeState.INTAKE){
//            return (timer.get() >= Intake.rollerIntakeDuration);
//        } else{
//        return (timer.get() >= Intake.rollerOuttakeDuration);
//        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!skip){
            coralIntake.setState(IntakeState.IDLE);
        } else if (coralIntake != null){
            coralIntake.setState(IntakeState.UNKNOWN);
        }
        super.end(interrupted);
    }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.onElevator;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.onElevator.CoralIntake;
import frc.robot.subsystems.onElevator.CoralIntake.CoralIntakeState;
import frc.utils.StormCommand;

public class CoralIntakeHold extends StormCommand {
    /**
     * Creates a new Intake.
     */
    private final CoralIntake coralIntake;
    private final Timer timer;

    public CoralIntakeHold(CoralIntake coralIntake) {
        this.coralIntake = coralIntake;
        timer = new Timer();
        addRequirements(coralIntake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
        timer.restart();
        coralIntake.setState(CoralIntakeState.GO_HOME);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 0.5;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        coralIntake.setState(CoralIntakeState.IDLE);
        super.end(interrupted);
    }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.onElevator;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.onElevator.CoralIntake;
import frc.robot.subsystems.onElevator.CoralIntake.IntakeState;
import frc.utils.StormCommand;

public class CoralIntakeHold extends StormCommand {
    private boolean skip;
    /**
     * Creates a new Intake.
     */
    private final CoralIntake coralIntake;
    private final CoralIntake.IntakePosition targetPosition;
    private final Timer timer;
    private double duration;

    public CoralIntakeHold(CoralIntake coralIntake, CoralIntake.IntakePosition targetPosition) {
        this.coralIntake = coralIntake;
        this.targetPosition = targetPosition;

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
        switch (targetPosition) {
            case OUTTAKE -> {
                coralIntake.setState(IntakeState.GO_HOME);
                duration = Constants.Intake.goHomeDuration;
            }
            case INTAKE -> {
                coralIntake.setState(IntakeState.HOLD_UP);
                duration = Constants.Intake.holdUpDuration;
            }
            default->{
                coralIntake.setState(IntakeState.IDLE);
                duration = 0;
            }
        }

        timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return skip || timer.get() > duration;

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

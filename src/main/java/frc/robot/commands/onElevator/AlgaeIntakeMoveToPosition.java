// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.onElevator;

import frc.robot.subsystems.onElevator.AlgaeIntake;
import frc.utils.StormCommand;

public class AlgaeIntakeMoveToPosition extends StormCommand {
    private final AlgaeIntake algaeIntake;
    protected final double targetPosition;

    public AlgaeIntakeMoveToPosition(AlgaeIntake algaeIntake, double position) {
        this.algaeIntake = algaeIntake;
        this.targetPosition = position;

        addRequirements(algaeIntake);
    }

    public AlgaeIntakeMoveToPosition(AlgaeIntake algaeIntake, AlgaeIntake.IntakeTarget target) {
        this(algaeIntake, target.getValue());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
        console("targetPosition = " + targetPosition);

        algaeIntake.setTargetPosition(targetPosition);
        algaeIntake.setState(AlgaeIntake.IntakeState.PID_MOTION);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return algaeIntake.isAtTarget();
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.onElevator;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.onElevator.CoralIntake;
import frc.robot.subsystems.onElevator.CoralIntake.IntakeState;
import frc.utils.StormCommand;

public class CoralIntakeGrip extends StormCommand {
    private boolean skip;
    private final CoralIntake coralIntake;
    private final Timer timer;
    private double duration;
    private RobotState robotState;
    private boolean earlyExit = false;

    public CoralIntakeGrip(CoralIntake coralIntake) {
        this.coralIntake = coralIntake;
        this.robotState = RobotState.getInstance();
        duration = Constants.Intake.holdRunDuration;

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
        earlyExit = robotState.isCoralSensorTriggered();
        timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        super.execute();

        if(robotState.isCoralSensorTriggered()) {
            timer.stop();
            timer.reset();
            coralIntake.setState(IntakeState.IDLE);
        } else {
            coralIntake.setState(IntakeState.GRIP_CORAL);
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return skip || (earlyExit ||
            timer.get() > duration);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!skip){
            coralIntake.setState(IntakeState.IDLE);
        } else if (coralIntake != null){
            coralIntake.setState(IntakeState.UNKNOWN);
        }
        timer.stop();
        timer.reset();

        super.end(interrupted);
    }
}

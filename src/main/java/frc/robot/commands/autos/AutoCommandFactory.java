package frc.robot.commands.autos;

import choreo.Choreo;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.Optional;

public class AutoCommandFactory {
    DrivetrainBase drivetrainBase;
    Optional trajectory = Choreo.loadTrajectory("test");


    public AutoCommandFactory(DrivetrainBase drivetrainBase) {
        this.drivetrainBase = drivetrainBase;
    }

    public Command buildChoreoCommand(Optional trajectory) {
        boolean openLoop = false;

        System.out.println("Control is " + (openLoop ? "open loop" : "pid controlled"));

        PIDController xController = new PIDController(openLoop ? 0 : 1,
            openLoop ? 0 : 0,
            openLoop ? 0 : 0
        );

        PIDController yController = new PIDController(openLoop ? 0 : 1,
            openLoop ? 0 : 0,
            openLoop ? 0 : 0
        );

        PIDController rotationController = new PIDController(openLoop ? 0 : 1,
            openLoop ? 0 : 0,
            openLoop ? 0 : 0
        );


//        return new AutoTrajectory(
//            trajectory.get().toString(),
//            trajectory,
//            ()->drivetrainBase.getPose(),
//            ()->drivetrainBase.declarePoseIsNow()
//        )
        return new Command() {
        };

    }


}

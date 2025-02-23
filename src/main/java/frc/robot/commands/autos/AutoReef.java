package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.joysticks.ReefscapeJoystick;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.StormCommand;
import frc.utils.vision.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class AutoReef extends StormCommand {
    DrivetrainBase drivetrainBase;
    VisionSubsystem visionSubsystem;
    ReefscapeJoystick joystick;
    int tagID = -1;
    Supplier<FieldConstants.Side> sideSupplier;
    Timer timer = new Timer();
    boolean flag = false;

    public AutoReef(DrivetrainBase drivetrainBase,
                    VisionSubsystem visionSubsystem,
                    ReefscapeJoystick joystick,
                    Supplier<FieldConstants.Side> side) {
        this.drivetrainBase = drivetrainBase;
        this.visionSubsystem = visionSubsystem;
        this.joystick = joystick;
        this.sideSupplier = side;


        addRequirements(drivetrainBase, visionSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
        super.initialize();
        FieldConstants.Side side;
        side = sideSupplier.get();
        if(side==null){
            System.out.println("side null?");
        }
        if (visionSubsystem.seesTag()) {
            tagID = visionSubsystem.getBestTag();
        }
        if (tagID != -1) {
            System.out.println("AutoReef: April Tag Seen!");
            Pose2d targetPose = FieldConstants.getReefTargetPose(side, tagID);
            Logger.recordOutput("Target Pose", targetPose);

            AutoAlign autoAlign = new AutoAlign(drivetrainBase, targetPose, joystick);
            autoAlign.schedule();
        } else {
            flag=true;
            System.out.println("No tag seen :(");
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
        @Override
    public boolean isFinished() {

//        return RobotState.getInstance().isAutonomousAligned() || flag;
        return timer.get()>5.0 || flag;
    }
}

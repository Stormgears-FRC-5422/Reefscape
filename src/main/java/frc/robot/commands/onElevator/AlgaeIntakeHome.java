package frc.robot.commands.onElevator;

import frc.robot.RobotState;
import frc.robot.subsystems.onElevator.AlgaeIntake;
import frc.robot.subsystems.onElevator.AlgaeIntake.IntakeState;
import frc.utils.StormCommand;

import static java.util.Objects.isNull;

public class AlgaeIntakeHome extends StormCommand {
    int counter;
    private boolean skip;

    private final AlgaeIntake intake;
    private RobotState robotState;
    // Don't allow homing if already homed. At least for now.
    // At this point the home will happen automatically at the beginning of auto or teleop
    // whichever is run first.
    
    public AlgaeIntakeHome(AlgaeIntake intake) {
        skip = robotState.getIntakeWristHasBeenHomed();
        this.intake = intake;
        robotState = RobotState.getInstance();
        
        if (!isNull(intake)) {
            addRequirements(intake);
        }
    }

    @Override
    public void initialize() {
        counter = 0;
        super.initialize();

        intake.setState(IntakeState.HOMING);
    }

    @Override
    public void execute() {
        super.execute();
        counter++;
    }

    @Override
    public boolean isFinished() {
        return counter>=50 || !skip;

    }

    @Override
    public void end(boolean interrupted) {
     if(skip){
        intake.setState(AlgaeIntake.IntakeState.UNKNOWN);
     }else{
        intake.setState(AlgaeIntake.IntakeState.HOME);

     }
        
            
        

        super.end(interrupted);
    }
}

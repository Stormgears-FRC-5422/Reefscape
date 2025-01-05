package frc.utils;

import edu.wpi.first.wpilibj2.command.Command;

public class StormCommand extends Command {
    public StormCommand() {
        super();
    }

    @Override
    public void initialize() {
        System.out.println(this.getClass() + " initialized");
    }


    @Override
    public void end(boolean interrupted) {
        System.out.println(this.getClass() + " ended");
    }
    
}

package frc.utils;

import edu.wpi.first.wpilibj2.command.Command;

public class StormCommand extends Command {
    protected String _name = "StormCommand";
    protected int _count = 0;

    public StormCommand() {
        _name = getClass().getName();
        _name = _name.substring(_name.lastIndexOf('.') + 1);
        console("created");
    }

    @Override
    public void initialize() {
        console("initialized");
    }

    @Override public void execute() {
        _count++;
    }

    @Override
    public void end(boolean interrupted) {
        console("ended: interrupted = " + interrupted);
    }

    public void console(String message) {
        System.out.println("Command " + _name + ": " + message);
    }

    public void console(String message, int iterations) {
        if (_count % iterations == 0) {
            console(message);
        }
    }
}

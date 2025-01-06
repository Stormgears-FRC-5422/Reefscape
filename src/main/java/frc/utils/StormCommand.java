package frc.utils;

import edu.wpi.first.wpilibj2.command.Command;

public class StormCommand extends Command {
    protected String m_name = "StormCommand";

    public StormCommand() {
        m_name = getClass().getName();
        m_name = m_name.substring(m_name.lastIndexOf('.') + 1);
        console("created");
    }

    @Override
    public void initialize() {
        console("initialized");
    }

    @Override
    public void end(boolean interrupted) {
        console("ended: interrupted = " + interrupted);
    }

    public void console(String message) {
        System.out.println("Command " + m_name + ": " + message);
    }
}

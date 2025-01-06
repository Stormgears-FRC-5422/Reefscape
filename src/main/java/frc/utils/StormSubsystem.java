package frc.utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StormSubsystem extends SubsystemBase {
    protected String m_name = "StormSubsystem";
    protected int iteration_count = 0;

    public StormSubsystem() {
        m_name = getClass().getName();
        m_name = m_name.substring(m_name.lastIndexOf('.') + 1);
        console("created");
    }

    @Override
    public void periodic() {
        iteration_count++;
    }

    public void console(String message) {
        System.out.println("Command " + m_name + ": " + message);
    }

    public void console(String message, int iterations) {
        if (iteration_count % iterations == 0) {
            console(message);
        }
    }
}

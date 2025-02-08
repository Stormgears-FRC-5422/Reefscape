package frc.robot.joysticks;

import edu.wpi.first.wpilibj.Joystick;

public class ReefscapeButtonBoard extends ReefscapeJoystick{

    Joystick m_joystickBoard1;
    Joystick m_joystickBoard2;

    public ReefscapeButtonBoard(int port){
        m_joystickBoard1 = new Joystick(port);
        m_joystickBoard2 = new Joystick (port+1);
        if (!m_joystickBoard2.getRawButton(1)){
            System.out.println("Switching ButtonBoard ports");
            m_joystickBoard1 = new Joystick(port+1);
            m_joystickBoard2 = new Joystick(port);
        }
        else{
            System.out.println("Not Switching ButtonBoard ports");
        }
    }

    public boolean getAutoManual(){
        return m_joystickBoard1.getRawButton(1);
    }

    public boolean elevatorLevel4(){
        return m_joystickBoard1.getRawButton(2);
    }

    public boolean elevatorLevel3(){
        return m_joystickBoard1.getRawButton(3);
    }

    public boolean elevatorLevel2(){
        return m_joystickBoard1.getRawButton(4);
    }

    public boolean elevatorLevel1(){
        return m_joystickBoard1.getRawButton(5);
    }

    public boolean getLeftRight(){
        return m_joystickBoard1.getRawButton(6);
    }

    public boolean coralIntake(){
        return m_joystickBoard1.getRawButton(7);
    }

    public boolean coralOuttake(){
        return m_joystickBoard1.getRawButton(10);
    }

    public boolean autoStation(){
        return m_joystickBoard1.getRawButton(8);
    }

    public boolean autoReef(){
        return m_joystickBoard1.getRawButton(9);
    }

    public boolean algaeIntake(){
        return m_joystickBoard2.getRawButton(12);
    }

    public boolean algaeOuttake(){
        return m_joystickBoard2.getRawButton(11);
    }

    public boolean autoProcessor(){
        return m_joystickBoard2.getRawButton(10);
    }

    public boolean autoAlgaeReef(){
        return m_joystickBoard2.getRawButton(9);
    }

    public boolean elevatorManual(){
        return m_joystickBoard2.getRawButton(8);
    }
}

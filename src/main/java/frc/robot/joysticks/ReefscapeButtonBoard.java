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
        System.out.println("joystick1, button 1");
        return m_joystickBoard1.getRawButton(1);
    }

    public boolean elevatorLevel1(){
        System.out.println("joystick1, button 2");
        return m_joystickBoard1.getRawButton(2);
    }

    public boolean elevatorLevel2(){
        System.out.println("joystick1, button 3");
        return m_joystickBoard1.getRawButton(3);
    }

    public boolean elevatorLevel3(){
        System.out.println("joystick1, button 4");
        return m_joystickBoard1.getRawButton(4);
    }

    public boolean elevatorLevel4(){
        System.out.println("joystick1, button 5");
        return m_joystickBoard1.getRawButton(5);
    }

    public boolean getLeftRight(){
        System.out.println("joystick1, button 6");
        return m_joystickBoard1.getRawButton(6);
    }

    public boolean coralIntake(){
        System.out.println("joystick1, button 7");
        return m_joystickBoard1.getRawButton(7);
    }

    public boolean coralOuttake(){
        System.out.println("joystick1, button 8");
        return m_joystickBoard1.getRawButton(8);
    }

    public boolean autoStation(){
        System.out.println("joystick1, button 9");
        return m_joystickBoard1.getRawButton(9);
    }

    public boolean autoReef(){
        System.out.println("joystick1, button 10");
        return m_joystickBoard1.getRawButton(10);
    }

    public boolean algaeIntake(){
        System.out.println("joystick2, button 12");
        return m_joystickBoard2.getRawButton(12);
    }

    public boolean algaeOuttake(){
        System.out.println("joystick2, button 11");
        return m_joystickBoard2.getRawButton(11);
    }

    public boolean autoProcessor(){
        System.out.println("joystick2, button 10");
        return m_joystickBoard2.getRawButton(10);
    }

    public boolean autoAlgaeReef(){
        System.out.println("joystick2, button 9");
        return m_joystickBoard2.getRawButton(9);
    }

    public boolean elevatorManual(){
        System.out.println("joystick2, button 8");
        return m_joystickBoard2.getRawButton(8);
    }
}

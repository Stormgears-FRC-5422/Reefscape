package frc.robot.joysticks;

import edu.wpi.first.wpilibj.Joystick;

public class ReefscapeButtonBoard extends ReefscapeJoystick {

    Joystick m_joystickBoard1;
    Joystick m_joystickBoard2;
    Joystick js1;
    Joystick js2;

    public ReefscapeButtonBoard(int port) {
        js1 = new Joystick(1);
        js2 = new Joystick(2);

        if (js1.getRawButton(1) && !js1.getRawButton(2) &&
            js2.getRawButton(2) && !js2.getRawButton(1)) {
            m_joystickBoard1 = js1;
            m_joystickBoard2 = js2;
        } else if (js1.getRawButton(2) && !js1.getRawButton(1) &&
            js2.getRawButton(1) && !js2.getRawButton(2)) {
            m_joystickBoard1 = js2;
            m_joystickBoard2 = js1;
        } else {
            System.out.println("**********");
            System.out.println("Bad joystick configuration!");
            System.out.println("**********");
        }
    }

    public boolean isAutoMode() {
        return m_joystickBoard1.getRawButton(12);
    }

    public boolean elevatorLevel4() {
        return m_joystickBoard1.getRawButton(11);
    }

    public boolean elevatorLevel3() {
        return m_joystickBoard1.getRawButton(3);
    }

    public boolean elevatorLevel2() {
        return m_joystickBoard1.getRawButton(4);
    }

    public boolean elevatorLevel1() {
        return m_joystickBoard1.getRawButton(5);
    }

    public boolean elevatorLevel4AutoRight() {
        return elevatorLevel4() && isAutoMode() && isRightReef();
    }

    public boolean elevatorLevel3AutoRight() {
        return elevatorLevel3() && isAutoMode() && isRightReef();
    }

    public boolean elevatorLevel2AutoRight() {
        return elevatorLevel2() && isAutoMode() && isRightReef();
    }

    public boolean elevatorLevel1AutoRight() {
        return elevatorLevel1() && isAutoMode() && isRightReef();
    }

    public boolean elevatorLevel4AutoLeft() {
        return elevatorLevel4() && isAutoMode() && !isRightReef();
    }

    public boolean elevatorLevel3AutoLeft() {
        return elevatorLevel3() && isAutoMode() && !isRightReef();
    }

    public boolean elevatorLevel2AutoLeft() {
        return elevatorLevel2() && isAutoMode() && !isRightReef();
    }

    public boolean elevatorLevel1AutoLeft() {
        return elevatorLevel1() && isAutoMode() && !isRightReef();
    }

    public boolean isRightReef() {
        return !m_joystickBoard1.getRawButton(6);
    }

    public boolean coralIntake() {
        return m_joystickBoard1.getRawButton(7);
    }

    public boolean coralOuttake() {
        return m_joystickBoard1.getRawButton(10);
    }

    public boolean autoStation() {
        return m_joystickBoard1.getRawButton(8);
    }

    public boolean autoReef() {
        return m_joystickBoard1.getRawButton(9);
    }

    public boolean algaeIntake() {
        return m_joystickBoard2.getRawButton(9);
    }

    public boolean algaeOuttake() {
        return m_joystickBoard2.getRawButton(10);
    }

    public boolean autoProcessor() {
        return m_joystickBoard2.getRawButton(11);
    }

    public boolean autoAlgaeReef() {
        return m_joystickBoard2.getRawButton(12);
    }

    // Move manual stick up or down to move elevator up and down manually
    public boolean elevatorUp() {
        double y = -m_joystickBoard2.getY();
        // There is a slight voltage bias that causes the joystick to report != 0 at rest
        // any actual motion sets it to -1.0 or 1.0, so we just need some reasonable number in the middle here
        return (Math.abs(y) > 0.5) && (y > 0);
    }

    public boolean elevatorDown() {
        double y = -m_joystickBoard2.getY();
        // There is a slight voltage bias that causes the joystick to report != 0 at rest
        // any actual motion sets it to -1.0 or 1.0, so we just need some reasonable number in the middle here
        return (Math.abs(y) > 0.5) && (y < 0);
    }
}

package frc.robot.joysticks;

public abstract class ReefscapeJoystick {
    public double getWpiX() {
        return 0;
    }

    public double getWpiY() {
        return 0;
    }

    public double getOmegaSpeed() {
        return 0;
    }

    public boolean getRobotRelative() {
        return true;
    }

    public boolean getTurbo() {
        return false;
    }

    public boolean zeroGyro() {
        return false;
    }

    public boolean zeroWheels() {
        return false;
    }

    public boolean coralIntake() {
        return false;
    }

    public boolean coralOuttake() {
        return false;
    }

    public boolean elevatorUp() {
        return false;
    }

    public boolean homeElevator() {
        return false;
    }

    public boolean elevatorDown() {
        return false;
    }

    public boolean getAutoManual() {
        return false;
    }

    public boolean elevatorLevel2() {
        return false;
    }

    public boolean elevatorLevel3() {
        return false;
    }

    public boolean elevatorLevel4() {
        return false;
    }

    public boolean getLeftRight() {
        return false;
    }

    public boolean autoStation() {
        return false;
    }

    public boolean autoReef() {
        return false;
    }

    public boolean algaeIntake() {
        return false;
    }

    public boolean algaeOuttake() {
        return false;
    }

    public boolean autoProcessor() {
        return false;
    }

    public boolean autoAlgaeReef() {
        return false;
    }

    public boolean elevatorManual() {
        return false;
    }
}

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

    public double getTurbo() {
        return 0.0;
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
    public boolean autoReef() {
        return false;
    }

    public boolean cancelAutoReef(){
        return false;
    }

    public boolean climb(){
        return false;
    }

    public boolean releaseClimb(){
        return false;
    }

    public boolean coralOuttake() {
        return false;
    }

    public boolean elevatorUp() {
        return false;
    }

    public boolean elevatorDown() {
        return false;
    }

    public boolean isAutoMode() {
        return false;
    }

    public boolean elevatorLevel1() {
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

    public boolean elevatorLevel1AutoRight() {
        return false;
    }

    public boolean elevatorLevel2AutoRight() {
        return false;
    }

    public boolean elevatorLevel3AutoRight() {
        return false;
    }

    public boolean elevatorLevel4AutoRight() {
        return false;
    }

    public boolean elevatorLevel1AutoLeft() {
        return false;
    }

    public boolean elevatorLevel2AutoLeft() {
        return false;
    }

    public boolean elevatorLevel3AutoLeft() {
        return false;
    }

    public boolean elevatorLevel4AutoLeft() {
        return false;
    }

    public boolean isRightReef() {
        return false;
    }

    public boolean autoStation() {
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

    public boolean elevatorTestPid() { return false; }

}

package frc.robot.joysticks;

public abstract class ReefscapeJoystick {
    public abstract double getWpiX();

    public abstract double getWpiY();

    public abstract double getOmegaSpeed();

    public abstract boolean getRobotRelative();

    public abstract double getTurbo();

    public abstract boolean zeroGyro();


    public abstract boolean zeroWheels();


}
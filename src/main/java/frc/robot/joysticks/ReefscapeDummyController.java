package frc.robot.joysticks;

public class ReefscapeDummyController extends ReefscapeJoystick {   
    ReefscapeDummyController(int port) {

    }
    public double getWpiX(){
        return 0.0;

    }


    public  double getWpiY(){
        return 0.0;
    }

    public  double getOmegaSpeed(){
        return 0.0;

    }

    public  boolean getRobotRelative(){
        return false;
    }

    public  double getTurbo(){return 0.0;}


    public  boolean zeroGyro(){return false;}


    public boolean zeroWheels(){return false;}


}
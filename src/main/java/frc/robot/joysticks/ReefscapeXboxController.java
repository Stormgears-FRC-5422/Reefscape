package frc.robot.joysticks;

import frc.utils.joysticks.StormXboxController;

public class ReefscapeXboxController extends ReefscapeJoystick {
    StormXboxController XboxController;

    ReefscapeXboxController(int port) {
        XboxController = new StormXboxController(port);
    }

    @Override
    public double getWpiX() {
        return XboxController.getWpiXSpeed();
    }

    @Override
    public double getWpiY() {
        return XboxController.getWpiYSpeed();
    }

    @Override
    public double getOmegaSpeed() {
        return XboxController.getOmegaSpeed();
    }

    @Override
    public boolean getRobotRelative() {
        return XboxController.getLeftTrigger() > 0.2;
    }

    @Override
    public boolean getTurbo() {
        return XboxController.getRightTrigger() > 0.2;
    }

    @Override
    public boolean zeroGyro() {
        return XboxController.getRightBumperIsPressed();
    }

    @Override
    public boolean coralIntake(){
        return XboxController.getXButtonIsHeld();
    }

    @Override
    public boolean coralOuttake(){
        return XboxController.getBButtonIsHeld();
    }

    // TODO: Uncomment after week 0, uses same buttons as diagnostic elevator
    /*
    @Override
    public boolean algaeIntake(){
        return XboxController.getYButtonIsHeld();
    }

    @Override
    public boolean algaeOuttake(){
        return XboxController.getAButtonIsHeld();
    }
     */

    @Override
    public boolean store(){
        return XboxController.getAButtonIsHeld();
    }

    @Override
    public boolean homeElevator(){
        return XboxController.getLeftLittleButtonIsHeld();
    }

    @Override
    public boolean elevatorLevel1(){
        return XboxController.getYButtonIsHeld();
    }
}

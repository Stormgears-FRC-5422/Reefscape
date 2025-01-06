package frc.robot.joysticks;

import frc.utils.joysticks.StormXboxController;

public class ReefscapeXboxController extends ReefscapeJoystick {
    StormXboxController XboxController;

    ReefscapeXboxController(int port) {
        XboxController = new StormXboxController(port);
    }

    public double getWpiX() {
        return XboxController.getWpiXSpeed();
    }

    public double getWpiY() {
        return XboxController.getWpiYSpeed();
    }

    public double getOmegaSpeed() {
        return XboxController.getOmegaSpeed();
    }

    public boolean getRobotRelative() {
        return XboxController.getLeftTrigger() > 0.2;
    }

    public boolean getTurbo() {
        return XboxController.getRightTrigger() > 0.2;
    }

    public boolean zeroGyro() {
        return XboxController.getRightBumperIsPressed();
    }
}

package frc.robot.joysticks;

public class ReefscapeJoystickFactory {
    public static ReefscapeJoystick instance;

    public static ReefscapeJoystick getInstance(String joystickType, int port) throws IllegalJoystickTypeException {
        System.out.println("Initializing " + joystickType + " as Joystick");
        switch (joystickType.toLowerCase()) {
            case "XboxController" -> instance = new ReefscapeXboxController(port);
//            case "dummy" -> instance = new ReefscapeDummyController(port);
            default -> throw new IllegalJoystickTypeException("Illegal Joystick Type: " + joystickType + " ---!");
        }
        return instance;
    }
}
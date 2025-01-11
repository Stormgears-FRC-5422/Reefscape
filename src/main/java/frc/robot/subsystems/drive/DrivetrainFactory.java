package frc.robot.subsystems.drive;

public class DrivetrainFactory {
    protected static DrivetrainBase instance;

    public static DrivetrainBase getInstance(String driveType) throws IllegalDriveTypeException {
        if (instance == null) {
            System.out.println("Initializing " + driveType);
            switch (driveType.toLowerCase()) {
                
                default -> throw new IllegalDriveTypeException("Illegal Drive Type: " + driveType);
            }
        }
        if (!driveType.equalsIgnoreCase("ctrdrive")) {
           
        }
        return instance;
    }
}

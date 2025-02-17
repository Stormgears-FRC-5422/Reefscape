package frc.robot.subsystems.drive;

import frc.robot.subsystems.drive.ctrGenerated.*;

public class DrivetrainFactory {
    protected static DrivetrainBase instance;

    public static DrivetrainBase getInstance(String driveType, String driveSubtype) throws IllegalDriveTypeException {
        if (instance == null) {
            System.out.println("Initializing " + driveType);
            switch (driveType.toLowerCase()) {
                case "diagnosticswerve" -> instance = new DiagnosticSwerve();
                case "ctrdrive" -> instance = new CTRDrivetrain(getTunerConstants(driveSubtype));
                case "akdrive" -> instance = new AKDriveTrain(getTunerConstants(driveSubtype));
                default -> throw new IllegalDriveTypeException("Illegal Drive Type: " + driveType);
            }
        }
        return instance;
    }

    // Right now there is only one kind of subtype. This could be more complicated if that changes.
    // We could just make a different get function for each driveType, but for now these should all work
    // with ctrdrive or akdrive types
    private static Class<?> getTunerConstants(String subtype) throws IllegalDriveTypeException {
        System.out.println("Collected constants for subtype: " + subtype);
        return switch (subtype.toLowerCase()) {
            case "ctrcrescendo" -> CrescendoTunerConstants.class;
            case "ctrreefscape" -> ReefscapeTunerConstants.class;
            case "ctrreeftorque" -> ReefscapeTorqueTunerConstants.class;
            case "ctrnovak" -> NovakTunerConstants.class;
            default -> throw new IllegalDriveTypeException("Illegal Drive subtype: " + subtype);
        };
    }

}

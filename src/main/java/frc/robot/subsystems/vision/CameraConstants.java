package frc.robot.subsystems.vision;

import frc.robot.Constants;

public class CameraConstants {

    public static StormLimelight[] ReefscapeLimelights = new StormLimelight[] {
        new StormLimelight(Constants.Vision.limelightID)
    };

    public static StormLimelight[] getReefscapeLimelights() {
        return ReefscapeLimelights;
    }
}

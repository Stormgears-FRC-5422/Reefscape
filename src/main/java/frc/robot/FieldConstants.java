package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.utils.vision.LimelightHelpers;

public class FieldConstants {
    public static boolean isAndyMark = false;

    public enum Side {
        LEFT, RIGHT
    }
    public static double redReefOffset = isAndyMark? -0.00508 : 0.0;

    /**
     * Method should NOT be used directly in commands
     */
    public static Pose2d getPoseTag(int id) {
        return switch (id) {
            case 1 ->
                new Pose2d(Constants.Vision.tag1X, Constants.Vision.tag1Y, new Rotation2d(Constants.Vision.tag1Rotation));
            case 2 ->
                new Pose2d(Constants.Vision.tag2X, Constants.Vision.tag2Y, new Rotation2d(Constants.Vision.tag2Rotation));
            case 3 ->
                new Pose2d(Constants.Vision.tag3X, Constants.Vision.tag3Y, new Rotation2d(Constants.Vision.tag3Rotation));
            case 4 ->
                new Pose2d(Constants.Vision.tag4X, Constants.Vision.tag4Y, new Rotation2d(Constants.Vision.tag4Rotation));
            case 5 ->
                new Pose2d(Constants.Vision.tag5X, Constants.Vision.tag5Y, new Rotation2d(Constants.Vision.tag5Rotation));
            case 6 ->
                new Pose2d(Constants.Vision.tag6X, Constants.Vision.tag6Y + redReefOffset, new Rotation2d(Constants.Vision.tag6Rotation));
            case 7 ->
                new Pose2d(Constants.Vision.tag7X, Constants.Vision.tag7Y + redReefOffset, new Rotation2d(Constants.Vision.tag7Rotation));
            case 8 ->
                new Pose2d(Constants.Vision.tag8X, Constants.Vision.tag8Y + redReefOffset, new Rotation2d(Constants.Vision.tag8Rotation));
            case 9 ->
                new Pose2d(Constants.Vision.tag9X, Constants.Vision.tag9Y + redReefOffset, new Rotation2d(Constants.Vision.tag9Rotation));
            case 10 ->
                new Pose2d(Constants.Vision.tag10X, Constants.Vision.tag10Y + redReefOffset, new Rotation2d(Constants.Vision.tag10Rotation));
            case 11 ->
                new Pose2d(Constants.Vision.tag11X, Constants.Vision.tag11Y + redReefOffset, new Rotation2d(Constants.Vision.tag11Rotation));
            case 12 ->
                new Pose2d(Constants.Vision.tag12X, Constants.Vision.tag12Y, new Rotation2d(Constants.Vision.tag12Rotation));
            case 13 ->
                new Pose2d(Constants.Vision.tag13X, Constants.Vision.tag13Y, new Rotation2d(Constants.Vision.tag13Rotation));
            case 14 ->
                new Pose2d(Constants.Vision.tag14X, Constants.Vision.tag14Y, new Rotation2d(Constants.Vision.tag14Rotation));
            case 15 ->
                new Pose2d(Constants.Vision.tag15X, Constants.Vision.tag15Y, new Rotation2d(Constants.Vision.tag15Rotation));
            case 16 ->
                new Pose2d(Constants.Vision.tag16X, Constants.Vision.tag16Y, new Rotation2d(Constants.Vision.tag16Rotation));
            case 17 ->
                new Pose2d(Constants.Vision.tag17X, Constants.Vision.tag17Y, new Rotation2d(Constants.Vision.tag17Rotation));
            case 18 ->
                new Pose2d(Constants.Vision.tag18X, Constants.Vision.tag18Y, new Rotation2d(Constants.Vision.tag18Rotation));
            case 19 ->
                new Pose2d(Constants.Vision.tag19X, Constants.Vision.tag19Y, new Rotation2d(Constants.Vision.tag19Rotation));
            case 20 ->
                new Pose2d(Constants.Vision.tag20X, Constants.Vision.tag20Y, new Rotation2d(Constants.Vision.tag20Rotation));
            case 21 ->
                new Pose2d(Constants.Vision.tag21X, Constants.Vision.tag21Y, new Rotation2d(Constants.Vision.tag21Rotation));
            case 22 ->
                new Pose2d(Constants.Vision.tag22X, Constants.Vision.tag22Y, new Rotation2d(Constants.Vision.tag22Rotation));
            default -> null;
        };
    }
    public static Pose2d getReefTargetPose(Side side, int id) {
        if ((id <= 11 && id >=6) || (id >= 17 && id <=22)) {
            if (side == Side.LEFT) {
                return getPoseTag(id).transformBy(new Transform2d(new Translation2d(0, -6.5), new Rotation2d(0)));
            } else if (side == Side.RIGHT) {
                return getPoseTag(id).transformBy(new Transform2d(new Translation2d(0, 6.5), new Rotation2d(0)));
            } else {
                return null;
            }
        } else {
            return null;
        }

    }
}

package frc.robot;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import frc.utils.vision.LimelightHelpers;
import org.littletonrobotics.junction.Logger;
import static frc.utils.Conversions.inchesToMeters;

public class FieldConstants {
    public static boolean isAndyMark = false;

    public enum Side {
        LEFT, RIGHT
    }
    public static double redReefOffset = isAndyMark? inchesToMeters(-0.2) : 0.0;

    /**
     * Method should NOT be used directly in commands
     * @param id the id of the tag
     */
    public static Pose2d getPoseTag(int id) {
        return switch (id) {
            case 1 ->
                new Pose2d(isAndyMark? inchesToMeters(656.98) : inchesToMeters(657.37), isAndyMark? inchesToMeters(24.73) : inchesToMeters(25.80), Rotation2d.fromDegrees(126.0));
            case 2 ->
                new Pose2d(isAndyMark? inchesToMeters(656.98) : inchesToMeters(657.37), isAndyMark? inchesToMeters(291.9) : inchesToMeters(291.20), Rotation2d.fromDegrees(234.0));
            case 3 ->
                new Pose2d(isAndyMark? inchesToMeters(452.4) : inchesToMeters(455.15), isAndyMark? inchesToMeters(316.21) : inchesToMeters(317.15), Rotation2d.fromDegrees(Constants.Vision.tag3Rotation));
            case 4 ->
                new Pose2d(inchesToMeters(365.20), inchesToMeters(241.64) + redReefOffset, Rotation2d.fromDegrees(Constants.Vision.tag4Rotation));
            case 5 ->
                new Pose2d(inchesToMeters(365.20), inchesToMeters(75.39) + redReefOffset, Rotation2d.fromDegrees(Constants.Vision.tag5Rotation));
            case 6 ->
                new Pose2d(inchesToMeters(530.49), inchesToMeters(130.17) + redReefOffset, Rotation2d.fromDegrees(Constants.Vision.tag6Rotation));
            case 7 ->
                new Pose2d(inchesToMeters(546.87), inchesToMeters(158.50) + redReefOffset, Rotation2d.fromDegrees(Constants.Vision.tag7Rotation));
            case 8 ->
                new Pose2d(inchesToMeters(530.49), inchesToMeters(186.83) + redReefOffset, Rotation2d.fromDegrees(Constants.Vision.tag8Rotation));
            case 9 ->
                new Pose2d(inchesToMeters(497.77), inchesToMeters(186.83) + redReefOffset, Rotation2d.fromDegrees(Constants.Vision.tag9Rotation));
            case 10 ->
                new Pose2d(inchesToMeters(481.39), inchesToMeters(158.5) + redReefOffset, Rotation2d.fromDegrees(Constants.Vision.tag10Rotation));
            case 11 ->
                new Pose2d(inchesToMeters(497.77), inchesToMeters(130.17) + redReefOffset, Rotation2d.fromDegrees(Constants.Vision.tag11Rotation));
            case 12 ->
                new Pose2d(isAndyMark? inchesToMeters(33.91) : inchesToMeters(33.51), isAndyMark? inchesToMeters(24.73) : inchesToMeters(25.8), Rotation2d.fromDegrees(Constants.Vision.tag12Rotation));
            case 13 ->
                new Pose2d(isAndyMark? inchesToMeters(33.91) : inchesToMeters(33.51), isAndyMark? inchesToMeters(291.9) : inchesToMeters(291.2), Rotation2d.fromDegrees(Constants.Vision.tag13Rotation));
            case 14 ->
                new Pose2d(inchesToMeters(325.68), isAndyMark? inchesToMeters(241.44) : inchesToMeters(241.64), Rotation2d.fromDegrees(Constants.Vision.tag14Rotation));
            case 15 ->
                new Pose2d(inchesToMeters(325.68), isAndyMark? inchesToMeters(75.19) : inchesToMeters(75.39), Rotation2d.fromDegrees(Constants.Vision.tag15Rotation));
            case 16 ->
                new Pose2d(isAndyMark? inchesToMeters(238.49) : inchesToMeters(235.73), isAndyMark? inchesToMeters(0.42) : inchesToMeters(-0.15), Rotation2d.fromDegrees(Constants.Vision.tag16Rotation));
            case 17 ->
                new Pose2d(inchesToMeters(160.39), isAndyMark? inchesToMeters(129.97) : inchesToMeters(130.17), Rotation2d.fromDegrees(Constants.Vision.tag17Rotation));
            case 18 ->
                new Pose2d(inchesToMeters(144), isAndyMark? inchesToMeters(158.3) : inchesToMeters(158.5), Rotation2d.fromDegrees(Constants.Vision.tag18Rotation));
            case 19 ->
                new Pose2d(inchesToMeters(160.39), isAndyMark? inchesToMeters(186.63) : inchesToMeters(186.83), Rotation2d.fromDegrees(Constants.Vision.tag19Rotation));
            case 20 ->
                new Pose2d(inchesToMeters(193.1), isAndyMark? inchesToMeters(186.63) : inchesToMeters(186.83), Rotation2d.fromDegrees(Constants.Vision.tag20Rotation));
            case 21 ->
                new Pose2d(inchesToMeters(209.49), isAndyMark? inchesToMeters(158.3) : inchesToMeters(158.5), Rotation2d.fromDegrees(Constants.Vision.tag21Rotation));
            case 22 ->
                new Pose2d(inchesToMeters(193.1), isAndyMark? inchesToMeters(129.97) : inchesToMeters(130.17), Rotation2d.fromDegrees(Constants.Vision.tag22Rotation));
            default -> null;
        };
    }
    /**
     * Method should NOT be used directly in commands
     * @param side the side of the reef
     * @param id the id of the tag
     */
    public static Pose2d getReefTargetPose(Side side, int id) {
        if ((id <= 11 && id >=6) || (id >= 17 && id <=22)) {
            if (side == Side.LEFT) {
                Logger.recordOutput("apriTagPose", getPoseTag(id));
                return getPoseTag(id).transformBy(new Transform2d(new Translation2d(0.5, -0.1978), new Rotation2d(Math.PI)));
            } else if (side == Side.RIGHT) {
                Logger.recordOutput("apriTagPose", getPoseTag(id));
                return getPoseTag(id).transformBy(new Transform2d(new Translation2d(0.5, 0.1978), new Rotation2d(Math.PI)));
            } else {
                return null;
            }
        } else {
            return null;
        }

    }
    public Pose2d getStationTargetPose(int id){
        if (id == 12 || id == 13 || id == 1 || id == 2) {
            return getPoseTag(id).transformBy(new Transform2d(new Translation2d(0.5,0), new Rotation2d(Math.PI)));
        } else {
            return null;
        }
    }
}

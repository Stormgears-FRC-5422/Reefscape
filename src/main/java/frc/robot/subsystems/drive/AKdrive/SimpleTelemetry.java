package frc.robot.subsystems.drive.AKdrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class SimpleTelemetry {
    /* NetworkTables instance */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher robotFieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final DoubleArrayPublisher visionFieldPub = table.getDoubleArrayTopic("visionPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    private final double[] robotPoseArray = new double[3];
    private final double[] visionPoseArray = new double[3];

    public SimpleTelemetry() {
        // Set the NetworkTables type for visualization in Glass
        fieldTypePub.set("Field2d");
    }

    /**
     * Publish the robot's pose to NetworkTables.
     *
     * @param robotPose The robot's current position and orientation on the field.
     * @param visionPose The robot's current position and orientation on the field.
     */
    public void telemeterize(Pose2d robotPose, Pose2d visionPose) {
        /* Convert pose to an array format (X, Y, Rotation in degrees) */
        robotPoseArray[0] = robotPose.getX();
        robotPoseArray[1] = robotPose.getY();
        robotPoseArray[2] = robotPose.getRotation().getDegrees();
        robotFieldPub.set(robotPoseArray);

        visionPoseArray[0] = visionPose.getX();
        visionPoseArray[1] = visionPose.getY();
        visionPoseArray[2] = visionPose.getRotation().getDegrees();
        visionFieldPub.set(visionPoseArray);
    }
}

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.networktables.TimestampedString;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

import java.time.Instant;
import java.time.Duration;
import java.util.Random;
import java.util.Random;


public class ComputerVision extends SubsystemBase {
    NetworkTable ntDataTable;
    NetworkTable ntCameraID;
    NetworkTableInstance nt_instance;

    double[] default_loc_from_RPi = {0.0, 0.0, 0.0, 0.0, 0.0};
    double[] actual_loc_from_RPi = {0.0, 0.0, 0.0, 0.0, 0.0};
    // double[] xyz = {0.0, 0.0, 0.0};
    // double[] tags = {0.0, 0.0};
    double x = 0;
    double y = 0;
    double z = 0;
    double tag1 = 0;
    double tag2 = 0;
    String strString;
    long actual_loc_server_time = 0;
    long actual_loc_timestamp = 0;
    int counter = 0;


    DoublePublisher xPub;
    DoublePublisher yPub;
    DoubleSubscriber zSub;

    double match_time = 0.0;
    boolean autonomous_mode = true;
    Instant start = Instant.now();
    Instant end = Instant.now();
    final int timeLimit = 5;
    NetworkTableEntry strCameraIDEntry;
    // long duration;
    IntegerPublisher cameraIDPublisher;
    int camID = 0;
    int totalCameras = 3;
    IntegerPublisher counterPublisher;

    DoubleArraySubscriber tsLocationSub;
    StringSubscriber strSub;
    StringSubscriber tsSub;
    TimestampedString tsString;
    TimestampedDoubleArray tsLocation;

    public ComputerVision() {
        this.nt_instance = NetworkTableInstance.getDefault();
        this.ntDataTable = nt_instance.getTable("datatable");
        // this.ntCameraID = nt_instance.getTable("CameraPublisher");

        // subscribe to the topic in "datatable" called "Y"
        // default value is 0
        // zSub = networkTable.getDoubleTopic("Y").subscribe(0.0);

        // publish to the topic in "datatable" called "Out"
        yPub = ntDataTable.getDoubleTopic("Y").publish();
        zSub = ntDataTable.getDoubleTopic("z").subscribe(0.0);
        strSub = ntDataTable.getStringTopic("apriltag_json").subscribe("apriltag_json not read.", PubSubOption.periodic(0.01));
        tsSub = ntDataTable.getStringTopic("apriltag_json2").subscribe("apriltag_json2 not read.", PubSubOption.periodic(0.001));
        // strCameraIDEntry = ntCameraID.getEntry("selected");
        // // Write a value to the string entry
        // strCameraIDEntry.setInteger(camID);
        tsLocationSub = ntDataTable.getDoubleArrayTopic(
            "/datatable/location_xyz").subscribe(default_loc_from_RPi, PubSubOption.periodic(0.01));

        cameraIDPublisher = ntDataTable.getIntegerTopic("Camera_ID").publish();
        cameraIDPublisher.set(camID);

        counterPublisher = ntDataTable.getIntegerTopic("counter").publish(PubSubOption.periodic(0.001));
        counterPublisher.set(counter);

    }

    public void periodic() {
        match_time = DriverStation.getMatchTime();
        autonomous_mode = DriverStation.isAutonomous();

        // Last 10 seconds of teleop mode will stream the ELP camera(camID=1). Auto and first 2'05" of
        // teleop will stream intake camera (camID=0).
        if (autonomous_mode) {
            camID = 0;
        }
        else {
            if (match_time > 10) {
                camID = 0;
            }
            else {
                camID = 1;
            }
        }

        // The following is to alternate the cameras every 5 seconds.
        // end = Instant.now();
        // // duration = Duration.between(start, end).getSeconds();

        // if (Duration.between(start, end).getSeconds() > timeLimit) {
        //     camID += 1;
        //     camID = camID % totalCameras;
        //     start = end;
        // }



        // Must run in every cycle. This value sends with a server clock / timestamp.
        cameraIDPublisher.set(camID);
        counterPublisher.set(counter);
        // nt_instance.flush();

        tsString = tsSub.getAtomic();
        System.out.printf("from RPi, value=%s, receive_in_serverT=%d, receive_in_clientT=%d\n\n", tsString.value, tsString.serverTime, tsString.timestamp);

        counter += 1;
        // System.out.println(counter);
    }
}

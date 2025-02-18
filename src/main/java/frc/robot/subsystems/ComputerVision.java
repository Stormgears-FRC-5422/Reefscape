package frc.robot.subsystems;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Random;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleEntry;
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
import frc.utils.StormSubsystem;

import java.util.Random;


public class ComputerVision extends StormSubsystem {
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
    String camID = "0";
    long actual_loc_server_time = 0;
    long actual_loc_timestamp = 0;
    int counter = 0;
    final int threshold = 3000;
    Random random = new Random();
    DoublePublisher xPub;
    DoublePublisher yPub;
    DoubleSubscriber zSub;
    NetworkTableEntry strCameraIDEntry;
    DoubleArraySubscriber tsLocationSub;
    StringSubscriber strSub;
    StringSubscriber tsSub;
    TimestampedString tsString;
    TimestampedDoubleArray tsLocation;

    public ComputerVision() {
        this.nt_instance = NetworkTableInstance.getDefault();
        this.ntDataTable = nt_instance.getTable("datatable");
        this.ntCameraID = nt_instance.getTable("CameraPublisher");

        // subscribe to the topic in "datatable" called "Y"
        // default value is 0
        // zSub = networkTable.getDoubleTopic("Y").subscribe(0.0);

        // publish to the topic in "datatable" called "Out"
        yPub = ntDataTable.getDoubleTopic("Y").publish();
        zSub = ntDataTable.getDoubleTopic("z").subscribe(0.0);
        strSub = ntDataTable.getStringTopic("apriltag_json").subscribe("apriltag_json not read.", PubSubOption.periodic(0.001));
        tsSub = ntDataTable.getStringTopic("apriltag_json2").subscribe("apriltag_json2 not read.", PubSubOption.periodic(0.001));
        strCameraIDEntry = ntCameraID.getEntry("selected");
        // Write a value to the string entry
        strCameraIDEntry.setString("0");
        tsLocationSub = ntDataTable.getDoubleArrayTopic(
            "/datatable/location_xyz").subscribe(default_loc_from_RPi, PubSubOption.periodic(0.001));
    }

    public void periodic() {
        // z = zSub.get();
        // if (z == 0){
        //     System.out.printf("default value from RPi, z=%f", z);}
        // else {
        //     System.out.printf("Get value from RPi, z=%f", z);
        // // System.out.println(z);
        // }

        // // xPub.set(x);
        y = random.nextDouble();
        yPub.set(y);

        if (counter % threshold < threshold * 0.5){
            if (camID == "1"){
                strCameraIDEntry.setString("0");
                camID = "0";
            }
        }
        else {
            if (camID == "0"){
                strCameraIDEntry.setString("1");
                camID = "1";
            }
        }

        nt_instance.flush();

        tsString = tsSub.getAtomic();
        System.out.printf("Received str from RPi, value=%s, receive_time_in_servertime=%d, receive_time_in_clienttime=%d\n\n", tsString.value, tsString.serverTime, tsString.timestamp);

        counter += 1;

    }
}

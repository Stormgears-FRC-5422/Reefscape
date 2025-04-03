package frc.robot.commands.vision;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.utils.StormCommand;

public class SwitchCams extends StormCommand {
    NetworkTable ntDataTable;
    IntegerSubscriber idSub;
    IntegerPublisher idPub;

    public SwitchCams() {
        ntDataTable = NetworkTableInstance.getDefault().getTable("datatable");
        idSub = ntDataTable.getIntegerTopic("CAMERA_ID").subscribe(0);
        idPub = ntDataTable.getIntegerTopic("CAMERA_ID").publish();
    }

    @Override
    public void initialize() {
        idPub.set(idSub.get() == 0 ? 1 : 0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

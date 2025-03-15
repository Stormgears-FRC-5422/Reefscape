package frc.robot.subsystems.stormnet;

import frc.robot.Constants;

public class BasicLidar extends StormNetSensor {
	private final short[] sensorDetails;
    private final int lidarCount;

	public BasicLidar(StormNetVoice voice) {
		super(voice);

		// two bytes per sensor
        // [0] is the distance in mm from sensor #0, [1] is sensor quality indicator, [2] starts sensor #1
        lidarCount = Constants.StormNet.lidarCount;
		sensorDetails = new short[lidarCount * 2];
		this.m_deviceString = voice.getDeviceString();
	}

	public void pollDetails() {
		fetchShorts("L", "LidarDistance", sensorDetails);
	}

    public double[] getDistances() {
        pollDetails();
        double[] distances = new double[lidarCount];

        for (int i = 0; i < lidarCount; i++) {
            distances[i] = ((double) sensorDetails[i*2] / 1000.0);
        }

        return distances;
    }


	public boolean getSensorQuality() {
		pollDetails();
		return( sensorDetails[1] != 0);
	}

}

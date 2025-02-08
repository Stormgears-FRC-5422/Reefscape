package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotState;
import frc.utils.StormSubsystem;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class Lights extends StormSubsystem {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private AddressableLEDBufferView m_left;
    private AddressableLEDBufferView m_middle;
    private AddressableLEDBufferView m_right;
    private AddressableLEDBufferView m_one;
    private LEDPattern defaultPattern;

    // Rainbow variables
    private Distance kLedSpacing;
    private LEDPattern m_rainbow;
    private LEDPattern m_scrollingRainbow;

    // Alliance variables
    private final RobotState m_robotState;
    private RobotState.StateAlliance m_alliance;

    // Constants
    // Total LEDs 24(Left strip) + 18(Middle strip) + 24(Right strip)
    private static final int LED_LENGTH = 18;
    private static final int LED_PORT = 0;

    public Lights() {
        m_led = new AddressableLED(LED_PORT);
        m_ledBuffer = new AddressableLEDBuffer(LED_LENGTH);

        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();

        // set color to alliance color at start
        m_robotState = RobotState.getInstance();
        m_alliance = m_robotState.getAlliance();
        setAllianceColor();
    }

    @Override
    public void periodic() {
        super.periodic();

        // Reset alliance color

        if (m_robotState.isCoralSensorTriggered()){
            setSolid(Color.kDarkGreen);
        }
        else{
            if (m_robotState.getAlliance() != m_alliance){
                m_alliance = m_robotState.getAlliance();
            }
            setAllianceColor();
        }
        setViews();
        // Write the data to the LED strip
        m_led.setData(m_ledBuffer);
    }

    public void setViews() {
        // TODO - make lots of these constants!
        //apply different patterns to each strip
        m_left = m_ledBuffer.createView(0, 10);
        //m_middle = m_ledBuffer.createView(24, 41);
        m_right = m_ledBuffer.createView(11, 16);
        m_one = m_ledBuffer.createView(17, 17);

        LEDPattern pattern1 = LEDPattern.solid(Color.kAqua);
        LEDPattern pattern2 = LEDPattern.solid(Color.kDeepPink);
        LEDPattern pattern3 = LEDPattern.solid(Color.kAzure);
        pattern1.applyTo(m_left);
        pattern2.applyTo(m_middle);
        pattern3.applyTo(m_right);


    }

    public void setRainbow() {
        m_rainbow = LEDPattern.rainbow(255, 128);
        kLedSpacing = Meters.of(1 / 120.0);
        m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.45), kLedSpacing);
        m_scrollingRainbow.applyTo(m_ledBuffer);
    }

    public void setAllianceColor(){
        switch(m_alliance){
            case RED -> {
                defaultPattern = LEDPattern.solid(Color.kRed);
            }
            case BLUE -> {
                defaultPattern = LEDPattern.solid(Color.kBlue);
            }
            default -> {
                defaultPattern = LEDPattern.solid(Color.kWhite);
            }
        }
        defaultPattern.applyTo(m_ledBuffer);
    }

    public void setSolid(Color color) {
        // Create an LED pattern that sets the entire strip to one color
        LEDPattern pattern = LEDPattern.solid(color);
        pattern.applyTo(m_ledBuffer);
    }

    public void setManually() {
        //Set a color for each LED manually
        m_ledBuffer.setRGB(0, 100,100,100);
    }

    public void elevatorLights(){
        m_one = m_ledBuffer.createView(LED_LENGTH-1, LED_LENGTH-1);
        //if (robotState.isElevatorHomed)
        LEDPattern pattern4 = LEDPattern.solid(Color.kLightGreen);
        pattern4.applyTo(m_one);

        //else
        LEDPattern pattern5 = LEDPattern.solid(Color.kRed);
        pattern5.applyTo(m_one);
    }
}

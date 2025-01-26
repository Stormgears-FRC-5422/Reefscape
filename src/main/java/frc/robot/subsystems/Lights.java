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
    private AddressableLEDBufferView m_right;

    //constants
    private static final int LED_LENGTH = 24; //Total LEDs
    private static final int VIEW_LENGTH = 12; //One Strip
    private static final int LED_PORT = 0;

    private Distance kLedSpacing;
    private LEDPattern m_rainbow;
    private LEDPattern m_scrollingRainbow;

    private final RobotState m_robotState;
    private RobotState.StateAlliance m_alliance;

    private LEDPattern defaultPattern;

    public Lights() {
        m_led = new AddressableLED(LED_PORT);
        m_ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
        m_left = m_ledBuffer.createView(0, 11);
        m_right = m_ledBuffer.createView(12, 23);
        m_led.setLength(m_ledBuffer.getLength());
        m_robotState = RobotState.getInstance();
        m_alliance = m_robotState.getAlliance();
        m_led.start();

        RobotState.StateAlliance alliance = m_robotState.getAlliance();
        if (alliance != m_alliance){
            m_alliance = alliance;
            setAlliancecolor();
        }
        //TODO: pass correct default color (red alliance or blue alliance)
        /*
        LEDPattern pattern1 = LEDPattern.solid(Color.kRed);
        LEDPattern pattern2 = LEDPattern.solid(Color.kBlue);
        pattern1.applyTo(m_left);
        pattern2.applyTo(m_right);

         */


    }

    @Override
    public void periodic() {
        super.periodic();
        //console("Lights On", 1000);

        //TODO: update color if intake sensor triggered - in intake Command?
        // if (sensor triggered)
        //  LEDPattern pattern = LEDPattern.solid(Color.kGreen);
        //  pattern.applyTo(m_ledBuffer);

        // Write the data to the LED strip
        m_scrollingRainbow.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);

    }

    public void setManually() {
        //Set a color for each LED manually
        m_ledBuffer.setRGB(0, 100,100,100);
    }

    public void setSolid(Color color) {
        // Create an LED pattern that sets the entire strip to one color
        //console("Solid LED");
        LEDPattern pattern = LEDPattern.solid(color);
    }

    public void setRainbow() {
        // Our LED strip has a density of 120 LEDs per meter
        //final Distance kLedSpacing = Meters.of(1 / 120.0);
        //console("Rainbow LED");
        //TODO: rainbow code
        m_rainbow = LEDPattern.rainbow(255, 128);
        kLedSpacing = Meters.of(1 / 120.0);
        m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
    }

    public void setAlliancecolor(){
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
    }

    //TODO: where to set this as a default command (sets default color of lights)
    // Color alliance = Color.kRed;
    // LEDPattern pattern = LEDPattern.solid(alliance);
    // setDefaultCommand(runPattern(pattern).withName("Alliance"));
    /*
    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(m_ledBuffer));
    }
     */
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.utils.StormSubsystem;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;

public class Lights extends StormSubsystem {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    //constants
    private static final int LED_LENGTH = 24; //Total LEDs
    private static final int VIEW_LENGTH = 12; //One Strip
    private static final int LED_PORT = 0;

    public Lights() {
        m_led = new AddressableLED(LED_PORT);
        m_ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
        //TODO: use buffer view to create 2 strips of length 12 - m_left:(0-11), m_right: (12-23)
        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();

        //TODO: pass correct default color (red alliance or blue alliance)
        LEDPattern pattern = LEDPattern.solid(Color.kRed);
        pattern.applyTo(m_ledBuffer);
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

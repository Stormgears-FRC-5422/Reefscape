package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.utils.StormSubsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class Lights extends StormSubsystem {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

    // Our LED strip has a density of 120 LEDs per meter
    private static final Distance kLedSpacing = Meters.of(1 / 120.0);

    // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
    // of 1 meter per second.
    private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

    public Lights() {
        m_led = new AddressableLED(0);
        m_ledBuffer = new AddressableLEDBuffer(24);
        m_led.setLength(m_ledBuffer.getLength());
        //m_ledBuffer.setRGB(0, 100,100,100);
        // Create an LED pattern that sets the entire strip to solid red
        LEDPattern red = LEDPattern.solid(Color.kRed);

        // Apply the LED pattern to the data buffer
        red.applyTo(m_ledBuffer);

// Write the data to the LED strip
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
}

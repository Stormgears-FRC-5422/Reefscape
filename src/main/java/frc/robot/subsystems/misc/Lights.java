package frc.robot.subsystems.misc;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.misc.BatteryMonitor.BatteryState;
import frc.robot.subsystems.onElevator.Elevator;
import frc.utils.StormSubsystem;

import static edu.wpi.first.units.Units.*;

public class Lights extends StormSubsystem {
    // Alliance variables
    private final RobotState m_robotState;
    Color RED_COLOR = Color.kRed;
    Color BLUE_COLOR = Color.kBlue;
    Color ORANGE_COLOR = Color.kOrangeRed;
    Color GREEN_COLOR = Color.kGreen;
    Color DARKGREEN_COLOR = Color.kDarkGreen;
    Color YELLOW_COLOR = Color.kYellow;
    Color LIGHTYELLOW_COLOR = Color.kLightYellow;
    Color PINK_COLOR = Color.kHotPink;
    Color NO_ALLIANCE_COLOR = ORANGE_COLOR;
    Color TAG_DETECTED_COLOR = YELLOW_COLOR;
    Color CORAL_STORED_COLOR = DARKGREEN_COLOR;
    Color AUTONOMOUS_ALIGNED_COLOR = GREEN_COLOR;
    Color ELEVATOR_HOMED_COLOR = LIGHTYELLOW_COLOR;
    Color ELEVATOR_HOLD_COLOR = PINK_COLOR;
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private boolean batteryPulse = false;
    private double batteryPulsePeriod;
    // Views
    private AddressableLEDBufferView m_left_top;
    private AddressableLEDBufferView m_left_bottom;
    private AddressableLEDBufferView m_right_top;
    private AddressableLEDBufferView m_right_bottom;
    private AddressableLEDBufferView m_middle;
    private RobotState.StateAlliance m_alliance;

    public Lights() {
        m_led = new AddressableLED(Constants.Lights.ledPort);
        m_ledBuffer = new AddressableLEDBuffer(Constants.Lights.ledLength);

        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();

        // split each strip into top and bottom views
        setViews();

        // set color to alliance color at start
        m_robotState = RobotState.getInstance();
        m_alliance = m_robotState.getAlliance();

        batteryPulsePeriod = Constants.Lights.batteryPulsePeriod;
    }

    @Override
    public void periodic() {
        super.periodic();

        this.batteryPulse = (m_robotState.getBatteryState() != BatteryState.GOOD);

        if (m_robotState.getAlliance() != m_alliance) {
            m_alliance = m_robotState.getAlliance();
        }

        console("in periodic. m_alliance is " + m_alliance, 500);

        setAllianceColor();

//        // Set alliance color by default to full strip
//        // if holding coral, change full strip to green
//        // if aligned to shoot coral, change top half to purple
//        if (m_robotState.isCoralSensorTriggered()) {
//            setSolid(CORAL_STORED_COLOR);
//        } else {
//            if (m_robotState.getAlliance() != m_alliance) {
//                m_alliance = m_robotState.getAlliance();
//            }
//            setAllianceColor();
//        }

        // Modify bottom view of strip based on april tag detection/alignment
        setAlignmentStatus();

//        // Modify top view of strip based on elevator status
//        setElevatorStatus();

        // Write the data to the LED strip
        m_led.setData(m_ledBuffer);
    }

    // Wrapper function to allow pulse effect to be applied to any pattern. Call this function rather
    // than calling pattern.applyTo()
    private <T extends LEDReader & LEDWriter> void patternApplyTo(LEDPattern basePattern, T view) {
        LEDPattern finalPattern = basePattern;

        if (batteryPulse) {
            finalPattern = basePattern.breathe(Seconds.of(batteryPulsePeriod));
        }

        finalPattern.applyTo(view);
    }

    public void setViews() {
        // split each strip into top and bottom views, so we can apply a different pattern to each section
        m_left_top = m_ledBuffer.createView(Constants.Lights.leftTopViewStart, Constants.Lights.leftTopViewEnd);
        m_left_bottom = m_ledBuffer.createView(Constants.Lights.leftBottomViewStart, Constants.Lights.leftBottomViewEnd);
        m_right_top = m_ledBuffer.createView(Constants.Lights.rightTopViewStart, Constants.Lights.rightTopViewEnd);
        m_right_bottom = m_ledBuffer.createView(Constants.Lights.rightBottomViewStart, Constants.Lights.rightBottomViewEnd);
        m_middle = m_ledBuffer.createView(Constants.Lights.middleViewStart, Constants.Lights.middleViewEnd);

    }

    public void setRainbow() {
        LEDPattern rainbow = LEDPattern.rainbow(255, 128);
        Distance kLedSpacing = Meters.of(1 / 120.0);
        LEDPattern m_scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.45), kLedSpacing);
        patternApplyTo(m_scrollingRainbow, m_ledBuffer);
    }

    public void setAllianceColor() {
        LEDPattern pattern;
        switch (m_alliance) {
            case RED -> {
                pattern = LEDPattern.solid(RED_COLOR);
            }
            case BLUE -> {
                pattern = LEDPattern.solid(BLUE_COLOR);
            }
            default -> {
                pattern = LEDPattern.solid(NO_ALLIANCE_COLOR);
            }
        }
        patternApplyTo(pattern, m_ledBuffer);
    }

    public void setSolid(Color color) {
        // Create an LED pattern that sets the entire view to one color
        LEDPattern pattern = LEDPattern.solid(color);
        patternApplyTo(pattern, m_ledBuffer);
    }

    public void setAlignmentStatus() {
        // Modify bottom view of strip based on alignment status
        LEDPattern pattern;
        if (m_robotState.isAligned()) {
            pattern = LEDPattern.solid(AUTONOMOUS_ALIGNED_COLOR);
            patternApplyTo(pattern, m_ledBuffer);
        } else if (m_robotState.isAprilTagDetected()) {
            pattern = LEDPattern.solid(TAG_DETECTED_COLOR);
            patternApplyTo(pattern, m_ledBuffer);
        } else if (m_robotState.isTeleopAligning()) {
            setRainbow();
        }
    }

    public void setElevatorStatus() {
        // Modify top view of strip based on elevator status
        isElevatorHomed();
//        isElevatorMoving();
    }

    public void isElevatorHomed() {
        if (m_robotState.elevatorHasBeenHomed()) {
            LEDPattern pattern = LEDPattern.solid(ELEVATOR_HOMED_COLOR);
            patternApplyTo(pattern, m_left_top);
            patternApplyTo(pattern, m_right_top);
        }
    }

    public void isElevatorMoving() {
        if (m_robotState.getElevatorState().equals(Elevator.ElevatorState.SIMPLE_MOTION)
            || m_robotState.getElevatorState().equals(Elevator.ElevatorState.PID_MOTION)) {
            setRainbow();
        }
    }

    public void isElevatorOnHoldAtPosition() {
        if (m_robotState.getElevatorState().equals(Elevator.ElevatorState.HOLD)) {
            LEDPattern pattern = LEDPattern.solid(ELEVATOR_HOLD_COLOR);
            patternApplyTo(pattern, m_left_top);
            patternApplyTo(pattern, m_right_top);
        }
    }
}

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.utils.StormSubsystem;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class Lights extends StormSubsystem {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    // Views
    private AddressableLEDBufferView m_left_top;
    private AddressableLEDBufferView m_left_bottom;
    private AddressableLEDBufferView m_right_top;
    private AddressableLEDBufferView m_right_bottom;

    // Patterns
    private LEDPattern defaultPattern;

    // Rainbow variables
    private Distance kLedSpacing;
    private LEDPattern m_rainbow;
    private LEDPattern m_scrollingRainbow;

    // Alliance variables
    private final RobotState m_robotState;
    private RobotState.StateAlliance m_alliance;

    Color RED_COLOR = scaleColor(Color.kRed, Constants.Lights.brightness);
    Color BLUE_COLOR = scaleColor(Color.kBlue, Constants.Lights.brightness);
    Color WHITE_COLOR = scaleColor(Color.kWhite, Constants.Lights.brightness);
    Color GREEN_COLOR = scaleColor(Color.kGreen, Constants.Lights.brightness);
    Color DARKGREEN_COLOR = scaleColor(Color.kDarkGreen, Constants.Lights.brightness);
    Color YELLOW_COLOR = scaleColor(Color.kYellow, Constants.Lights.brightness);
    Color LIGHTYELLOW_COLOR = scaleColor(Color.kLightYellow, Constants.Lights.brightness);
    Color PINK_COLOR = scaleColor(Color.kHotPink, Constants.Lights.brightness);

    Color NO_ALLIANCE_COLOR = WHITE_COLOR;
    Color TAG_DETECTED_COLOR = YELLOW_COLOR;
    Color SENSOR_TRIGGERED_COLOR = DARKGREEN_COLOR;
    Color AUTONOMOUS_ALIGNED_COLOR = GREEN_COLOR;
    Color ELEVATOR_HOMED_COLOR = LIGHTYELLOW_COLOR;
    Color ELEVATOR_HOLD_COLOR = PINK_COLOR;

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
        setAllianceColor();
    }

    @Override
    public void periodic() {
        super.periodic();

        // Set alliance color by default to full strip
        // if holding coral, change full strip to green
        // if aligned to shoot coral, change top half to purple
        if (m_robotState.isCoralSensorTriggered()){
            setSolid(SENSOR_TRIGGERED_COLOR);
        }
        else{
            if (m_robotState.getAlliance() != m_alliance){
                m_alliance = m_robotState.getAlliance();
            }
            setAllianceColor();
        }

        // Modify bottom view of strip based on april tag detection/alignment
        setAlignmentStatus();

        // Modify top view of strip based on elevator status
        setElevatorStatus();

        // Write the data to the LED strip
        m_led.setData(m_ledBuffer);
    }

    private static Color scaleColor(Color c, double s) {
        return new Color(MathUtil.clamp((int) (s * c.red), 0, 255),
            MathUtil.clamp((int) (s * c.green), 0, 255),
            MathUtil.clamp((int) (s * c.blue), 0, 255));
    }

    public void setViews() {
        // split each strip into top and bottom views, so we can apply a different pattern to each section
        m_left_top = m_ledBuffer.createView(Constants.Lights.leftTopViewStart, Constants.Lights.leftTopViewEnd);
        m_left_bottom = m_ledBuffer.createView(Constants.Lights.leftBottomViewStart, Constants.Lights.leftBottomViewEnd);
        m_right_top = m_ledBuffer.createView(Constants.Lights.rightTopViewStart, Constants.Lights.rightTopViewEnd);
        m_right_bottom = m_ledBuffer.createView(Constants.Lights.rightBottomViewStart, Constants.Lights.rightBottomViewEnd);
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
                defaultPattern = LEDPattern.solid(RED_COLOR);
            }
            case BLUE -> {
                defaultPattern = LEDPattern.solid(BLUE_COLOR);
            }
            default -> {
                defaultPattern = LEDPattern.solid(NO_ALLIANCE_COLOR);
            }
        }
        defaultPattern.applyTo(m_ledBuffer);
    }

    public void setSolid(Color color) {
        // Create an LED pattern that sets the entire view to one color
        LEDPattern pattern = LEDPattern.solid(color);
        pattern.applyTo(m_ledBuffer);
    }

    public void setAlignmentStatus() {
        // Modify bottom view of strip based on alignment status
        if (m_robotState.isAprilTagDetected()) {
            LEDPattern pattern = LEDPattern.solid(TAG_DETECTED_COLOR);
            pattern.applyTo(m_left_bottom);
            pattern.applyTo(m_right_bottom);
        }

        if (m_robotState.isAutonomousAligned()) {
            LEDPattern pattern = LEDPattern.solid(AUTONOMOUS_ALIGNED_COLOR);
            pattern.applyTo(m_left_bottom);
            pattern.applyTo(m_right_bottom);
        }
    }

    public void setElevatorStatus() {
        // Modify top view of strip based on elevator status
        isElevatorHomed();
        isElevatorMoving();
    }

    public void isElevatorHomed() {
        if (m_robotState.elevatorHasBeenHomed()) {
            LEDPattern pattern = LEDPattern.solid(ELEVATOR_HOMED_COLOR);
            pattern.applyTo(m_left_top);
            pattern.applyTo(m_right_top);
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
            pattern.applyTo(m_left_top);
            pattern.applyTo(m_right_top);
        }
    }
}

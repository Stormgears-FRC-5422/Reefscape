package frc.robot.subsystems.misc;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import frc.utils.StormSubsystem;

public class ColorSensorFactory extends StormSubsystem {
    private final ColorSensorV3 colorSensor;
    private final ColorMatch colorMatch;

    private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);

    public ColorSensorFactory(I2C.Port port) {
        colorSensor = new ColorSensorV3(port);
        colorMatch = new ColorMatch();

        colorMatch.addColorMatch(kBlueTarget);
        colorMatch.addColorMatch(kRedTarget);
    }

    public String checkColor() {

        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);

        String colorString;

        if (match.color == kBlueTarget) {
            colorString = "Blue";
        } else if (match.color == kRedTarget) {
            colorString = "Red";
        } else {
            colorString = "Blank";
        }

        // TODO use smart dashboard

        console("Color: " + colorString);
        console("Red: " + detectedColor.red);
        console("Green: " + detectedColor.green);
        console("Blue: " + detectedColor.blue);

        return colorString;
    }

    public int getProximity() {
        return colorSensor.getProximity();
    }

    public double getConfidence(ColorMatchResult match) {return match.confidence;}
}

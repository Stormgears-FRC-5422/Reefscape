// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.utils.StormSubsystem;

public class ColorSensor extends StormSubsystem {
    private final ColorSensorV3 colorSensor;
    private final ColorMatch colorMatch;

    private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);

    public ColorSensor() {
        colorSensor = new ColorSensorV3(I2C.Port.kMXP);
        colorMatch = new ColorMatch();

        colorMatch.addColorMatch(kBlueTarget);
        colorMatch.addColorMatch(kRedTarget);
    }

    @Override
    public void periodic() {
        super.periodic();
        Color detectedColor = colorSensor.getColor();

        String colorString;
        ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);

        int proximity = colorSensor.getProximity();

        if (match.color == kBlueTarget) {
            colorString = "Blue";
        } else if (match.color == kRedTarget) {
            colorString = "Red";
        } else {
            colorString = "Blank";
        }

        // TODO find actual values that limit wrong color detection

//        if (match.color == kBlueTarget && proximity > 250 && match.confidence > 0.8 && detectedColor.blue > 0.3) {
//            colorString = "Blue";
//        } else if (match.color == kRedTarget && proximity > 250 && match.confidence > 0.8 && detectedColor.red > 0.3) {
//            colorString = "Red";
//        } else {
//            colorString = "Blank";
//        }

        // TODO use smart dashboard

        console("Red: " + detectedColor.red);
        console("Green: " + detectedColor.green);
        console("Blue: " + detectedColor.blue);
        console("Color: " + colorString);
        console("Confidence: " + match.confidence);
        console("Proximity: " + proximity);
    }
}

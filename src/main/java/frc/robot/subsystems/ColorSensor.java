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
    private final I2C.Port i2cPort;

    private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);

    public ColorSensor() {
        i2cPort = I2C.Port.kOnboard;
        colorSensor = new ColorSensorV3(i2cPort);
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

        if (match.color == kBlueTarget) {
            colorString = "Blue";
        } else if (match.color == kRedTarget) {
            colorString = "Red";
        } else {
            colorString = "Blank";
        }

        int proximity = colorSensor.getProximity();

        // TODO use smart dashboard

        console("Red: " + detectedColor.red, 25);
        console("Green: " + detectedColor.green, 25);
        console("Blue: " + detectedColor.blue, 25);
        console("Color: " + colorString, 25);
        console("Confidence: " + match.confidence, 25);
        console("Proximity: " + proximity, 25);
    }
}

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
import frc.robot.subsystems.ColorSensorTemplate;

public class ColorSensor extends StormSubsystem {

    private final ColorSensorV3 colorSensorLeft;
    private final ColorSensorV3 colorSensorRight;
    private final ColorMatch colorMatch;

    private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);

    private final ColorSensorTemplate colorSensor;

    public ColorSensor() {

        colorSensorLeft = new ColorSensorV3(I2C.Port.kMXP);
        colorSensorRight = new ColorSensorV3(I2C.Port.kOnboard);
        colorMatch = new ColorMatch();

        colorMatch.addColorMatch(kBlueTarget);
        colorMatch.addColorMatch(kRedTarget);

        colorSensor = new ColorSensorTemplate(I2C.Port.kMXP);
    }

    @Override
    public void periodic() {
        super.periodic();

        console(colorSensor.checkColor());
        Color detectedColorLeft = colorSensorLeft.getColor();
        Color detectedColorRight = colorSensorRight.getColor();

        String colorStringLeft;
        String colorStringRight;
        ColorMatchResult matchLeft = colorMatch.matchClosestColor(detectedColorLeft);
        ColorMatchResult matchRight = colorMatch.matchClosestColor(detectedColorRight);

//        int proximity = colorSensor.getProximity();

        if (matchLeft.color == kBlueTarget) {
            colorStringLeft = "Blue";
        } else if (matchLeft.color == kRedTarget) {
            colorStringLeft = "Red";
        } else {
            colorStringLeft = "Blank";
        }

        if (matchRight.color == kBlueTarget) {
            colorStringRight = "Blue";
        } else if (matchRight.color == kRedTarget) {
            colorStringRight = "Red";
        } else {
            colorStringRight = "Red";
        }

        // TODO find actual values that limit wrong color detection

        // TODO use smart dashboard

//        console("Color left: " + colorStringLeft);
//        console("Color right: " + colorStringRight);
//
//        console("Red right: " + detectedColorRight.red);
//        console("Green right: " + detectedColorRight.green);
//        console("Blue right: " + detectedColorRight.blue);
//
//        console("Red left: " + detectedColorLeft.red);
//        console("Green left: " + detectedColorLeft.green);
//        console("Blue left: " + detectedColorLeft.blue);


//        console("Confidence: " + match.confidence);
//        console("Proximity: " + proximity);
    }
}


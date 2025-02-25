// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.misc;

import edu.wpi.first.wpilibj.I2C;
import frc.utils.StormSubsystem;

public class ColorSensor extends StormSubsystem {

    private final ColorSensorFactory colorSensorLeft;
    private final ColorSensorFactory colorSensorRight;

    public ColorSensor() {
        colorSensorLeft = new ColorSensorFactory(I2C.Port.kMXP);
        colorSensorRight = new ColorSensorFactory(I2C.Port.kOnboard);
    }

    @Override
    public void periodic() {
        super.periodic();
        console(colorSensorLeft.checkColor());
        console(colorSensorRight.checkColor());
    }
}


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import frc.utils.StormSubsystem;

public class ColorSensor extends StormSubsystem {

    private final ColorSensorFactory colorSensor;

    public ColorSensor() {
        colorSensor = new ColorSensorFactory(I2C.Port.kMXP);
    }

    @Override
    public void periodic() {
        super.periodic();
        console(colorSensor.checkColor());
    }
}


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.Debug;

public final class Main {
    private Main() {}

    public static void main(String... args) {

        try {
            RobotBase.startRobot(Robot::new);
        } catch (Exception e) {
            System.out.println("Unhandled exception in robot code!!");
            System.out.println(e.getMessage());
            e.printStackTrace();
        }

        // We only get here if we caught in un
        if (Debug.debug && Debug.holdCrash) {
            try {
                while (true) {
                    Thread.sleep(1000);
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}

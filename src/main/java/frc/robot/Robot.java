// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.robot.Constants.Toggles;
import frc.robot.RobotState.*;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;
    private RobotState state;
    private int iteration = 0;

    public Robot() {
        // Most people want to call getInstance(). This is only created once, here.
        state = RobotState.createInstance();
        Logger.recordMetadata("ProjectName", Constants.robotName); // Set a metadata value
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("GitDirty", switch (BuildConstants.DIRTY) {
            case 0 -> "All changes committed";
            case 1 -> "Uncommitted changes";
            default -> "Unknown";
        });

        switch (state.getSimMode()) {
            case REAL:
                console("This is a REAL robot");
                // TOOD - figure out what happens if there is no USB drive plugged in?
                // I think /home/lvuser/logs?
                Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
                //new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
                break;
            case AKIT_REPLAY:
                console("This is an ADVANTAGE KIT REPLAY robot");
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
                Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
                break;
            case AKIT_SIM:
                console("This is an ADVANTAGE KIT SIMULATION robot");
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIMULATION:
                console("This is a SIMULATION robot");
                break;
        }

        if (Toggles.useAdvantageKit) {
            Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
        }

        try {
            robotContainer = new RobotContainer();
        } catch (Exception e) {
            robotContainer = null;
            console("can't create RobotContainer. Eating the following exception:");
            //noinspection CallToPrintStackTrace
            e.printStackTrace();
        }
    }

    @Override
    public void robotPeriodic() {
        iteration++;
        Threads.setCurrentThreadPriority(true, 99);
        CommandScheduler.getInstance().run();
        Threads.setCurrentThreadPriority(false, 10);
    }

    @Override
    public void disabledInit() {
        console("DisabledInit");
        state.setPeriod(StatePeriod.DISABLED);
		if (robotContainer != null) {
		    robotContainer.updateAlliance();
		}
    }

    @Override
    public void disabledPeriodic() {
        if (iteration % 50 == 0) {
		    if (robotContainer != null) {
                robotContainer.updateAlliance();
			}
        }
    }

    @Override
    public void disabledExit() {
        console("DisabledExit");
    }

    @Override
    public void autonomousInit() {
        console("AutoInit");
        state.setPeriod(StatePeriod.AUTONOMOUS);
        if (robotContainer != null) {
            robotContainer.updateAlliance();
            autonomousCommand = robotContainer.getAutonomousCommand();

        }

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
		console("AutoExit");
    }

    @Override
    public void teleopInit() {
        console("TeleopInit");
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        console("TeleopExit");
    }

    @Override
    public void testInit() {
        console("TestInit");
        CommandScheduler.getInstance().cancelAll();
        if (robotContainer != null) {
            robotContainer.updateAlliance();
        }
        state.setPeriod(StatePeriod.TEST);
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
        console("TestExit");
    }
    @Override
    public void simulationInit() {
        console("SimulationInit");
        if (robotContainer != null) {
            robotContainer.updateAlliance();
        }
    }

    @Override
    public void simulationPeriodic() {
    }

    public void console(String message) {
        System.out.println("Robot : " + message);
    }

    public void console(String message, int iterations) {
        if (iteration % iterations == 0) {
            console(message);
        }
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.robot2025;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.lang.reflect.Field;

import org.frogforce503.lib.subsystem.VirtualSubsystem;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.robot2025.Constants.Bot;
import org.frogforce503.robot2025.fields.FieldConfig.Venue;
import org.frogforce503.robot2025.hardware.RobotHardware;
import org.frogforce503.robot2025.hardware.RobotHardwareCompBot;
import org.frogforce503.robot2025.hardware.RobotHardwarePracticeBot;
import org.frogforce503.robot2025.hardware.RobotHardwareProgrammingBot;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private final double loopOverrunWarningTimeout = 0.2;

  private RobotContainer robotContainer;
  public static RobotHardware bot;

  // Configuration Parameters
  private final Bot selectedBot = Bot.SimBot;
  private final Venue selectedVenue = Venue.Shop;
  
  /*
   * Robot Constructor 
   */
  public Robot() {
    Constants.setRobotType(selectedBot);
    
    bot =
        switch (Constants.getRobot()) {
            case SimBot, CompBot -> new RobotHardwareCompBot();
            case PracticeBot -> new RobotHardwarePracticeBot();
            case ProgrammingBot -> new RobotHardwareProgrammingBot();
        };
    
    bot.initializeConstants();
  }
 
 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "FF2025_" + Constants.getRobot().name().toUpperCase()); // Set a metadata value

    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Adjust loop overrun warning timeout
    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(loopOverrunWarningTimeout);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }

    // Disable alerts for disconnected controllers
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure brownout voltage
    RobotController.setBrownoutVoltage(6.0);

    // Initialize RobotContainer
    robotContainer = new RobotContainer(selectedVenue);

    // Switch thread to high priority to improve loop timing
    // Threads.setCurrentThreadPriority(true, 10);

    // Warmup auto chooser
    robotContainer.warmupAutoChooser();
  }

  @Override
  public void robotPeriodic() {
    LoggedTracer.reset();
    
    // Run virtual subsystems
    VirtualSubsystem.periodicAll();

    // Run command scheduler
    CommandScheduler.getInstance().run();
    LoggedTracer.record("CommandScheduler");

    SmartDashboard.putData(CommandScheduler.getInstance());

    // Record cycle time
    LoggedTracer.record("RobotPeriodic");
  }

  @Override
  public void autonomousInit() {
    robotContainer.startAuto();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    robotContainer.stopClaw(); // Make sure coral doesn't eject incase state goes to EJECT_CORAL
    robotContainer.cleanupAutoChooser();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    robotContainer.coastAfterAutoEnd();
  }

  @Override
  public void disabledPeriodic() {
    robotContainer.updateAutoChooser();
    robotContainer.seedWristPosition();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
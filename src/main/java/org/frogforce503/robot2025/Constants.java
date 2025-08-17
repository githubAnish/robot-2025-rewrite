package org.frogforce503.robot2025;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import lombok.Setter;
import edu.wpi.first.wpilibj.RobotBase;

/** This class contains global configuration describing the current robot and runtime mode. */
public final class Constants {
  public static final double loopPeriodSecs = 0.02;

  public static final boolean useAllianceFlipping = false;
  
  @Setter private static Bot robotType = Bot.CompBot;

  @SuppressWarnings("resource")
  public static Bot getRobot() {
    if (!disableHAL && RobotBase.isReal() && robotType == Bot.SimBot) {
        new Alert("Invalid robot selected, using competition robot as default.", AlertType.kError)
            .set(true);
        robotType = Bot.CompBot;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (robotType) {
      case ProgrammingBot, PracticeBot, CompBot -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SimBot -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum Bot {
    CompBot, PracticeBot, ProgrammingBot, SimBot
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (robotType == Bot.SimBot) {
        System.err.println("Cannot deploy, invalid robot selected: " + robotType);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (robotType != Bot.CompBot) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }
}
package org.frogforce503.robot.commands;

import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.climber.Climber.ClimberState;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivotConstants;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimbingCommands {
    private ClimbingCommands() {}

    public static Command setPivotDown(Superstructure superstructure) { // Climb step 1
        return Commands.runOnce(() -> superstructure.getIntakePivot().setAngle(IntakePivotConstants.minAngle));
    }

    public static Command bringPivotUp(Superstructure superstructure) { // Climb step 2
        return Commands.runOnce(() -> superstructure.getIntakePivot().runVolts(0.2 * RobotController.getBatteryVoltage()));
    }

    public static Command fastWind(Climber climber) { // Climb step 3, when button pressed
        return Commands.runOnce(() -> climber.setCurrentState(ClimberState.FAST_WIND));
    }

    public static Command slowWind(Superstructure superstructure, Climber climber) { // Climb step 3, when button released
        return
            Commands.parallel(
                Commands.runOnce(() -> superstructure.getIntakePivot().stop()),
                Commands.runOnce(() -> climber.setCurrentState(ClimberState.SLOW_WIND)));
    }
}

package org.frogforce503.robot2025.commands;

import org.frogforce503.robot2025.subsystems.climber.Climber;
import org.frogforce503.robot2025.subsystems.climber.Climber.ClimberState;
import org.frogforce503.robot2025.subsystems.superstructure.intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimbingCommands {
    private ClimbingCommands() {}

    public static Command setPivotDown(Intake intake) { // Climb step 1
        return Commands.runOnce(() -> intake.runOpenLoop(-0.2, 0.0));
    }

    public static Command bringPivotUp(Intake intake) { // Climb step 2
        return Commands.runOnce(() -> intake.runOpenLoop(0.2, 0.0));
    }

    public static Command fastWind(Climber climber) {
        return Commands.runOnce(() -> climber.setCurrentState(ClimberState.FAST_WIND));
    }
}

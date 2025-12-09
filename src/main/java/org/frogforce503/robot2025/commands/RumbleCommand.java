package org.frogforce503.robot2025.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleCommand extends StartEndCommand {
    public RumbleCommand(CommandXboxController joystick, double minRumbleStrength, double maxRumbleStrength) {
        super(
            () -> joystick.setRumble(RumbleType.kBothRumble, minRumbleStrength),
            () -> joystick.setRumble(RumbleType.kBothRumble, maxRumbleStrength));
    }

    public RumbleCommand(CommandXboxController joystick) {
        this(joystick, 0.0, 1.0);
    }
}
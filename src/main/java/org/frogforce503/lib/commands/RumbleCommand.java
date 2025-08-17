package org.frogforce503.lib.commands;

import org.frogforce503.lib.math.Range;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleCommand extends StartEndCommand {
    public RumbleCommand(CommandXboxController joystick, Range rumbleStrengths) {
        super(
            () -> joystick.setRumble(RumbleType.kBothRumble, rumbleStrengths.max()),
            () -> joystick.setRumble(RumbleType.kBothRumble, rumbleStrengths.min()));
    }

    public RumbleCommand(CommandXboxController joystick) {
        this(
            joystick,
            new Range(0.0, 1.0));
    }
}
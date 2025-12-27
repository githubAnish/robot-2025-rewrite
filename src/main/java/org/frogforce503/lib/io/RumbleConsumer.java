package org.frogforce503.lib.io;

import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleConsumer implements BiConsumer<RumbleType, Double> {
    private final CommandXboxController xboxController;

    public RumbleConsumer(CommandXboxController xboxController) {
        this.xboxController = xboxController;
    }

    @Override
    public void accept(RumbleType t, Double u) {
        xboxController.setRumble(t, u);
    }
}

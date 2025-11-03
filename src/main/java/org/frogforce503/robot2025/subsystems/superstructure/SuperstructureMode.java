package org.frogforce503.robot2025.subsystems.superstructure;

import org.frogforce503.lib.reefscape.Gamepiece;

import lombok.Getter;

public enum SuperstructureMode {
    CORAL_INTAKE(Gamepiece.CORAL),
    L1(Gamepiece.CORAL),
    L2(Gamepiece.CORAL),
    L3(Gamepiece.CORAL),
    L4(Gamepiece.CORAL),

    ALGAE_GROUND(Gamepiece.ALGAE),
    ALGAE_HANDOFF(Gamepiece.ALGAE),
    ALGAE_PLUCK_HIGH(Gamepiece.ALGAE),
    ALGAE_PLUCK_LOW(Gamepiece.ALGAE),
    PROCESSOR(Gamepiece.ALGAE),
    BARGE(Gamepiece.ALGAE);

    @Getter private Gamepiece gamepiece;

    private SuperstructureMode(Gamepiece gamepiece) {
        this.gamepiece = gamepiece;
    }
}
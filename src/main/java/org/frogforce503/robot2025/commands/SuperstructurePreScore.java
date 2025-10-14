package org.frogforce503.robot2025.commands;

import java.util.HashMap;

import org.frogforce503.lib.util.FFSelectCommand;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure.Mode;

public class SuperstructurePreScore extends FFSelectCommand<Mode> {
    public SuperstructurePreScore(Superstructure superstructure) {
        super(
            new HashMap<>() {{
                // Coral
                put(Mode.L1, superstructure.preScoreL1());
                put(Mode.L2, superstructure.preScoreL2());
                put(Mode.L3, superstructure.preScoreL3());
                put(Mode.L4, superstructure.preScoreL4());
            }},
            superstructure::getCurrentMode);
    }
}
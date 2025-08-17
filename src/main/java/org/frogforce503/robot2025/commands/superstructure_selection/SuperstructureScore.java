package org.frogforce503.robot2025.commands.superstructure_selection;

import java.util.HashMap;

import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure.Mode;

import edu.wpi.first.wpilibj2.command.SelectCommand;

public class SuperstructureScore extends SelectCommand<Mode> {
    public SuperstructureScore(Superstructure superstructure) {
        super(
            new HashMap<>() {{
                // Coral
                put(Mode.L1, superstructure.scoreL1());
                put(Mode.L2, superstructure.scoreL2());
                put(Mode.L3, superstructure.scoreL3());
                put(Mode.L4, superstructure.scoreL4());

                // Algae
                put(Mode.PROCESSOR, superstructure.scoreProcessor());
                put(Mode.BARGE, superstructure.scoreBarge());
            }},
            superstructure::getCurrentMode);
    }
}
package org.frogforce503.robot2025.commands.superstructure_selection;

import java.util.HashMap;
import java.util.function.Supplier;

import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure.Mode;

import edu.wpi.first.wpilibj2.command.SelectCommand;

public class SuperstructurePreScore extends SelectCommand<Mode> {
    public SuperstructurePreScore(Superstructure superstructure, Supplier<Mode> modeSupplier) {
        super(
            new HashMap<>() {{
                // Coral
                put(Mode.L1, superstructure.preScoreL1());
                put(Mode.L2, superstructure.preScoreL2());
                put(Mode.L3, superstructure.preScoreL3());
                put(Mode.L4, superstructure.preScoreL4());
            }},
            modeSupplier);
    }
}
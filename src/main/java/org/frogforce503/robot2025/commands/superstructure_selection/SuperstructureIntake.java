package org.frogforce503.robot2025.commands.superstructure_selection;

import java.util.HashMap;
import java.util.function.Supplier;

import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure.Mode;

import edu.wpi.first.wpilibj2.command.SelectCommand;

public class SuperstructureIntake extends SelectCommand<Mode> {
    public SuperstructureIntake(Superstructure superstructure, Supplier<Mode> superstructureModeSupplier) {
        super(
            new HashMap<>() {{
                // Coral
                put(Mode.CORAL_INTAKE, superstructure.intakeCoral());
    
                // Algae
                put(Mode.ALGAE_GROUND, superstructure.intakeAlgaeFromGround());
                put(Mode.ALGAE_HANDOFF, superstructure.intakeAlgaeFromHandoff());
                put(Mode.ALGAE_PLUCK_HIGH, superstructure.pluckHighAlgae());
                put(Mode.ALGAE_PLUCK_LOW, superstructure.pluckLowAlgae());
            }},
            superstructureModeSupplier);
    }
}
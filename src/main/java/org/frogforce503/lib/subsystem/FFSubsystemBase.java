package org.frogforce503.lib.subsystem;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A thin wrapper around WPILib's {@link SubsystemBase} class to create command-based subsystems with common FF boilerplate methods. */
public abstract class FFSubsystemBase extends SubsystemBase {
    @Override
    public abstract void periodic();

    /** Override this method to apply to your tunable subsystem. */
    public BooleanConsumer tuningExecutor() {
        return
            tuningEnabled -> {};
    };

    public abstract boolean atGoal();

    public abstract Command stop();
    public abstract Command runManual(double output);
}
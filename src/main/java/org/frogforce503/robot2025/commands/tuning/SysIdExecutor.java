package org.frogforce503.robot2025.commands.tuning;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysIdExecutor {
    private final Consumer<Voltage> consumer;
    private final SysIdRoutine routine;

    public SysIdExecutor(FFSubsystemBase subsystem, Consumer<Voltage> consumer) {
        // Create routine
        this.consumer = consumer;
        this.routine =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    Volts.of(1).per(Second),
                    Volts.of(7),
                    Seconds.of(10),
                    state -> Logger.recordOutput(subsystem.getName() + "/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    consumer,
                    null, // No log consumer, since data is recorded by AdvantageKit
                    subsystem));
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return
            Commands.run(() -> consumer.accept(Volts.of(0.0)))
                .withTimeout(1.0)
                .andThen(routine.quasistatic(direction));
    }
    
    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(
        Velocity<VoltageUnit> rampRate,
        Voltage stepVoltage,
        Time timeout,
        SysIdRoutine.Direction direction
    ) {
        return
            Commands.run(() -> consumer.accept(Volts.of(0.0)))
                .withTimeout(1.0)
                .andThen(routine.dynamic(direction));
    }
}

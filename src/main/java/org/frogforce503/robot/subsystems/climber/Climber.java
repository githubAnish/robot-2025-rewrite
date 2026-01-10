package org.frogforce503.robot.subsystems.climber;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.robot.subsystems.climber.ClimberIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import lombok.Setter;

public class Climber extends FFSubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    // Constants
    private final Debouncer holdDebouncer = new Debouncer(0.1, DebounceType.kRising);

    // Control
    @Setter private ClimberState currentState = ClimberState.IDLE;
    private boolean holdRequested = false;

    public enum ClimberState {
        IDLE,
        SLOW_WIND,
        FAST_WIND,
        HOLD
    }

    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        if (currentThresholdForHoldMet()) {
            holdRequested = true;
        }

        if (holdRequested && currentState != ClimberState.SLOW_WIND) {
            currentState = ClimberState.HOLD;
        }

        switch (currentState) {
            case IDLE:
                break;

            case SLOW_WIND:
                io.runOpenLoop(0.05);
                break;

            case FAST_WIND:
                io.runOpenLoop(1.0);
                break;

            case HOLD:
                io.runOpenLoop(0.3);
                break;
        }

        Logger.recordOutput("Climber/State", currentState.name());

        // Record cycle time
        LoggedTracer.record("Climber");
    }

    private boolean currentThresholdForHoldMet() {
        return holdDebouncer.calculate(
            inputs.data.statorCurrentAmps() > 63);
    }

    // Actions
    @Override
    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

    @Override
    public void stop() {
        io.stop();
    }
}
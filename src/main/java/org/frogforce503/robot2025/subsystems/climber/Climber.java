package org.frogforce503.robot2025.subsystems.climber;

import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.LimitSwitchIO;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.LimitSwitchIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import lombok.Setter;

public class Climber extends FFSubsystemBase {
    private final ClimberIO climberIO;
    private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

    private final LimitSwitchIO limitSwitchIO;
    private final LimitSwitchIOInputsAutoLogged limitSwitchInputs = new LimitSwitchIOInputsAutoLogged();

    // Constants
    private final Debouncer currentHoldDebouncer = new Debouncer(0.1, DebounceType.kRising);

    // Control
    @Setter private ClimberState currentState = ClimberState.IDLE;
    private boolean holdRequested = false;

    public enum ClimberState {
        IDLE,
        SLOW_WIND,
        FAST_WIND,
        HOLD
    }

    public Climber(ClimberIO climberIO, LimitSwitchIO limitSwitchIO) {
        this.climberIO = climberIO;
        this.limitSwitchIO = limitSwitchIO;
    }

    @Override
    public void periodic() {
        super.periodic();

        climberIO.updateInputs(climberInputs);
        Logger.processInputs("Climber/Winch", climberInputs);

        limitSwitchIO.updateInputs(limitSwitchInputs);
        Logger.processInputs("Climber/LimitSwitch", limitSwitchInputs);

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
                climberIO.runVolts(0.05);
                break;

            case FAST_WIND:
                climberIO.runVolts(1.0);
                break;

            case HOLD:
                climberIO.runVolts(0.3);
                break;
        }

        Logger.recordOutput("Climber/State", currentState.name());

        // Record cycle time
        LoggedTracer.record("Climber");
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        climberIO.setBrakeMode(enabled);
    }

    @Override
    public void stop() {
        climberIO.stop();
    }

    public void runOpenLoop(double output) {
        currentState = ClimberState.IDLE;
        climberIO.runOpenLoop(output);
    }

    private boolean currentThresholdForHoldMet() {
        return
            currentHoldDebouncer.calculate(
                climberInputs.data.statorCurrentAmps() > 63);
    }
}
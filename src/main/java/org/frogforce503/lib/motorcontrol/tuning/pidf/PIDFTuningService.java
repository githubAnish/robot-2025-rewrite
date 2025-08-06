package org.frogforce503.lib.motorcontrol.tuning.pidf;

import org.frogforce503.lib.motorcontrol.tuning.TuningService;
import org.frogforce503.lib.util.LoggedTunableNumber;

public class PIDFTuningService implements TuningService<PIDFConfig> {
    private PIDFTuningConfig config;
    private PIDFConfig oldConfig;

    public PIDFTuningService(String key, PIDFConfig config) {
        this.config = new PIDFTuningConfig(
            new LoggedTunableNumber(key + "/kP", config.kP()),
            new LoggedTunableNumber(key + "/kI", config.kI()),
            new LoggedTunableNumber(key + "/kD", config.kD()),
            new LoggedTunableNumber(key + "/kS", config.kS()),
            new LoggedTunableNumber(key + "/kG", config.kG()),
            new LoggedTunableNumber(key + "/kV", config.kV()),
            new LoggedTunableNumber(key + "/kA", config.kA())
        );

        this.oldConfig = config;
    }

    @Override
    public void setTuning(boolean enabled) {
        if (enabled != isTuningEnabled()) { // Update only if tuning state changes
            config.kP().setTuningMode(enabled);
            config.kI().setTuningMode(enabled);
            config.kD().setTuningMode(enabled);
            config.kS().setTuningMode(enabled);
            config.kG().setTuningMode(enabled);
            config.kV().setTuningMode(enabled);
            config.kA().setTuningMode(enabled);
        }
    }

    @Override
    public boolean isTuningEnabled() {
        return
            config.kP().isTuningMode() &&
            config.kI().isTuningMode() &&
            config.kD().isTuningMode() &&
            config.kS().isTuningMode() &&
            config.kG().isTuningMode() &&
            config.kV().isTuningMode() &&
            config.kA().isTuningMode();
    }

    @Override
    public PIDFConfig getUpdatedConfig() {
        if (config.kP().hasChanged(hashCode()) ||
            config.kI().hasChanged(hashCode()) ||
            config.kD().hasChanged(hashCode()) ||
            config.kS().hasChanged(hashCode()) ||
            config.kG().hasChanged(hashCode()) ||
            config.kV().hasChanged(hashCode()) ||
            config.kA().hasChanged(hashCode())
        ) {
            return new PIDFConfig(
                config.kP().get(),
                config.kI().get(),
                config.kD().get(),
                config.kS().get(),
                config.kG().get(),
                config.kV().get(),
                config.kA().get());
        }
        
        return oldConfig;
    }
}
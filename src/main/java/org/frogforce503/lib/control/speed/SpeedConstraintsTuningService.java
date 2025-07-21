package org.frogforce503.lib.control.speed;

import org.frogforce503.lib.control.TuningService;
import org.frogforce503.lib.util.LoggedTunableNumber;

public class SpeedConstraintsTuningService implements TuningService<SpeedConstraintsConfig> {
    private SpeedConstraintsTuningConfig config;
    private SpeedConstraintsConfig oldConfig;

    public SpeedConstraintsTuningService(String key, SpeedConstraintsConfig config) {
        this.config = new SpeedConstraintsTuningConfig(
            new LoggedTunableNumber(key + "/MaxVelocityMetersPerSec", config.maxVelocityMetersPerSec()),
            new LoggedTunableNumber(key + "/MaxAccelerationMetersPerSec2", config.maxAccelerationMetersPerSec2())
        );

        this.oldConfig = config;
    }

    @Override
    public void setTuning(boolean enabled) {
        if (enabled != isTuningEnabled()) { // Update only if tuning state changes
            config.maxVelocityMetersPerSec().setTuningMode(enabled);
            config.maxAccelerationMetersPerSec2().setTuningMode(enabled);
        }
    }

    @Override
    public boolean isTuningEnabled() {
        return
            config.maxVelocityMetersPerSec().isTuningMode() &&
            config.maxAccelerationMetersPerSec2().isTuningMode();
    }

    @Override
    public SpeedConstraintsConfig getUpdatedConfig() {
        if (config.maxVelocityMetersPerSec().hasChanged(hashCode()) ||
            config.maxAccelerationMetersPerSec2().hasChanged(hashCode())
        ) {
            return new SpeedConstraintsConfig(
                config.maxVelocityMetersPerSec().get(),
                config.maxAccelerationMetersPerSec2().get());
        }
        
        return oldConfig;
    }
}
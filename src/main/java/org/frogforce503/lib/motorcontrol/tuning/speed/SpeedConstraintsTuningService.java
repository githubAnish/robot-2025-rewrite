package org.frogforce503.lib.motorcontrol.tuning.speed;

import org.frogforce503.lib.motorcontrol.tuning.TuningService;
import org.frogforce503.lib.util.LoggedTunableNumber;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class SpeedConstraintsTuningService implements TuningService<Constraints> {
    private SpeedConstraintsTuningConfig config;
    private Constraints oldConfig;

    public SpeedConstraintsTuningService(String key, Constraints config) {
        this.config = new SpeedConstraintsTuningConfig(
            new LoggedTunableNumber(key + "/MaxVelocityMetersPerSec", config.maxVelocity),
            new LoggedTunableNumber(key + "/MaxAccelerationMetersPerSec2", config.maxAcceleration)
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
    public Constraints getUpdatedConfig() {
        if (config.maxVelocityMetersPerSec().hasChanged(hashCode()) ||
            config.maxAccelerationMetersPerSec2().hasChanged(hashCode())
        ) {
            return new Constraints(
                config.maxVelocityMetersPerSec().get(),
                config.maxAccelerationMetersPerSec2().get());
        }
        
        return oldConfig;
    }
}
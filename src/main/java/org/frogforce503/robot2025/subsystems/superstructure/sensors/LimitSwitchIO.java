package org.frogforce503.robot2025.subsystems.superstructure.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface LimitSwitchIO {
    @AutoLog
    class LimitSwitchIOInputs {
        public LimitSwitchIOData data = new LimitSwitchIOData(false);
    }   

    record LimitSwitchIOData(boolean pressed) {}

    default void updateInputs(LimitSwitchIOInputs inputs) {}
}
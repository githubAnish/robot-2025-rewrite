package org.frogforce503.robot2025.subsystems.superstructure.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface LimitSwitchIO {
    @AutoLog
    class DigitalIOInputs {
        public DigitalIOData data = new DigitalIOData(false);
    }   

    record DigitalIOData(boolean pressed) {}

    default void updateInputs(DigitalIOInputs inputs) {}
}
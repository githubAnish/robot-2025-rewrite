package org.frogforce503.robot2025.subsystems.superstructure.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface CoralSensorIO {
    @AutoLog
    class CoralSensorIOInputs {
        public CoralSensorIOData data = new CoralSensorIOData(false, false);
    }   

    record CoralSensorIOData(
        boolean upperBeamBreakTriggered,
        boolean lowerBeamBreakTriggered) {}

    default void updateInputs(CoralSensorIOInputs inputs) {}
}

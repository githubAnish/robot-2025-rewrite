package org.frogforce503.robot2025.subsystems.superstructure.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface CoralSensorIO {
    @AutoLog
    class CoralSensorIOInputs {
        public CoralSensorIOData data = new CoralSensorIOData(false, false);
    }   

    record CoralSensorIOData(
        boolean upperTriggered,
        boolean lowerTriggered) {}

    default void updateInputs(CoralSensorIOInputs inputs) {}
}

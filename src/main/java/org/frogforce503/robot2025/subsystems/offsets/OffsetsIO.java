package org.frogforce503.robot2025.subsystems.offsets;

import org.littletonrobotics.junction.AutoLog;

public interface OffsetsIO {
    @AutoLog
    class OffsetsIOInputs {
        public OffsetsIOData data = new OffsetsIOData(false, "", "", 0.0);
    }

    record OffsetsIOData(
        boolean tuning,
        String branch,
        String direction,
        double value) {}

    default void updateInputs(OffsetsIOInputs inputs) {}

    default void setValue(double value) {}
}
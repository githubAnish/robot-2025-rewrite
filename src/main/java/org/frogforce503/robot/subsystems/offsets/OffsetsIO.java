package org.frogforce503.robot.subsystems.offsets;

import org.littletonrobotics.junction.AutoLog;

public interface OffsetsIO {
    @AutoLog
    class OffsetsIOInputs {
        public boolean tuning = false;
        public String branch = "";
        public String direction = "";
        public double value = 0.0;
    }

    default void updateInputs(OffsetsIOInputs inputs) {}

    default void setValue(double value) {}
}
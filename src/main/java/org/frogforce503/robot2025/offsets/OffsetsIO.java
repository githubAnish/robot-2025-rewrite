package org.frogforce503.robot2025.offsets;

import org.frogforce503.robot2025.commands.coral_score_reef.Branch;
import org.frogforce503.robot2025.commands.coral_score_reef.ReefSide;
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

    default void setTuning(boolean set) {}

    default void setSelectedBranchId(ReefSide side, Branch branch) {}

    default void setDirection(Direction direction) {}

    default void setValue(double value) {}
}
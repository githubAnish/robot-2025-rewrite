package org.frogforce503.robot2025.commands.coral_score_reef;

import java.util.Map;
import java.util.function.Supplier;

import org.frogforce503.lib.util.ProximityUtil;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.offsets.Offset;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public enum Branch {
    LEFT {
        @Override
        public Supplier<Pose2d> getTarget(Drive drive, FieldInfo field, Map<String, Offset> offsets) {
            return
                () ->
                    calculateTarget(
                        drive,
                        field,
                        offsets);
        }
    },
    RIGHT {
        @Override
        public Supplier<Pose2d> getTarget(Drive drive, FieldInfo field, Map<String, Offset> offsets) {
            return
                () ->
                    calculateTarget(
                        drive,
                        field,
                        offsets);
        }
    };

    public abstract Supplier<Pose2d> getTarget(Drive drive, FieldInfo field, Map<String, Offset> offsets);

    protected Pose2d calculateTarget(Drive drive, FieldInfo field, Map<String, Offset> offsets) {
        ReefSide goal = ProximityUtil.getClosestReefSide(drive, field);
        
        return
            goal
                .getTarget(field)
                .get()
                .plus(
                    offsets
                        .get(this.name() + "_" + goal.name())
                        .toTransform2d());
    }
}
package org.frogforce503.robot2025.commands.coral_score_reef;

import java.util.Map;
import java.util.function.Supplier;

import org.frogforce503.lib.util.ProximityService;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.offsets.Offset;

import edu.wpi.first.math.geometry.Pose2d;

public enum Branch {
    LEFT {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field, ProximityService proximityService, Map<String, Offset> offsets) {
            return
                () ->
                    calculateTarget(
                        field,
                        proximityService,
                        offsets);
        }
    },
    RIGHT {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field, ProximityService proximityService, Map<String, Offset> offsets) {
            return
                () ->
                    calculateTarget(
                        field,
                        proximityService,
                        offsets);
        }
    };

    public abstract Supplier<Pose2d> getTarget(FieldInfo field, ProximityService proximityService, Map<String, Offset> offsets);

    protected Pose2d calculateTarget(FieldInfo field, ProximityService proximityService, Map<String, Offset> offsets) {
        ReefSide goal = proximityService.getClosestReefSide();
        
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
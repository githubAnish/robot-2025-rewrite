package org.frogforce503.lib.reefscape;

import java.util.function.Supplier;

import org.frogforce503.robot2025.FieldInfo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum Station {
    LEFT {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return
                () ->
                    new Pose2d(
                        field.getLeftStation(),
                        Rotation2d.fromDegrees(field.onRedAlliance() ? 125 : -55));
        }
    },
    RIGHT {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return
                () ->
                    new Pose2d(
                        field.getRightStation(),
                        Rotation2d.fromDegrees(field.onRedAlliance() ? -125 : 55));
        }
    };

    public abstract Supplier<Pose2d> getTarget(FieldInfo field);
}
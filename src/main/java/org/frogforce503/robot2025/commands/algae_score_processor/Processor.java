package org.frogforce503.robot2025.commands.algae_score_processor;

import java.util.function.Supplier;

import org.frogforce503.robot2025.fields.FieldInfo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum Processor {
    RED {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return
                () ->
                    new Pose2d(
                        field.getConfiguration().RedProc,
                        new Rotation2d(Math.PI / 2));
        }
    },
    BLUE {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return
                () ->
                    new Pose2d(
                        field.getConfiguration().BlueProc,
                        new Rotation2d(-Math.PI / 2));
        }
    };

    public abstract Supplier<Pose2d> getTarget(FieldInfo field);
}
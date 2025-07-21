package org.frogforce503.robot2025.commands.cage;

import java.util.function.Supplier;

import org.frogforce503.robot2025.fields.FieldInfo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum Cage {
    LEFT {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return
                () ->
                    new Pose2d(
                        field.getLeftCage(),
                        field.onRedAlliance()
                            ? Rotation2d.kPi
                            : Rotation2d.kZero);
        }
    },
    CENTER {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return
                () ->
                    new Pose2d(
                        field.getCenterCage(),
                        field.onRedAlliance()
                            ? Rotation2d.kPi
                            : Rotation2d.kZero);
        }
    },
    RIGHT {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return
                () ->
                    new Pose2d(
                        field.getRightCage(),
                        field.onRedAlliance()
                            ? Rotation2d.kPi
                            : Rotation2d.kZero);
        }
    };

    public abstract Supplier<Pose2d> getTarget(FieldInfo field);
}
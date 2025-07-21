package org.frogforce503.robot2025.commands.algae_score_barge;

import java.util.function.Supplier;

import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public enum Barge {
    ALLIANCE_SIDE {
        @Override
        public Supplier<Pose2d> getTarget(Drive drive, FieldInfo field) {
            return
                () -> {
                    final boolean red = field.onRedAlliance();
                    final double offset = Units.inchesToMeters(14.25 - 15);
                    
                    double bargeX =
                        red
                            ? field.getConfiguration().RedInitLine - offset
                            : field.getConfiguration().BlueInitLine + offset;

                    return new Pose2d(
                        new Translation2d(bargeX, drive.getCurrentPose().getY()),
                        red ? Rotation2d.kZero : Rotation2d.kPi);
                };
        }
    },
    OTHER_ALLIANCE_SIDE {
        @Override
        public Supplier<Pose2d> getTarget(Drive drive, FieldInfo field) {
            return
                () -> {
                    final boolean red = field.onRedAlliance();
                    final double offset = Units.inchesToMeters(14.25 - 15);
                    
                    double bargeX =
                        red
                            ? field.getConfiguration().BlueInitLine + offset
                            : field.getConfiguration().RedInitLine - offset;

                    return new Pose2d(
                        new Translation2d(bargeX, drive.getCurrentPose().getY()),
                        red ? Rotation2d.kPi : Rotation2d.kZero);
                };
        }
    };

    public abstract Supplier<Pose2d> getTarget(Drive drive, FieldInfo field);
}
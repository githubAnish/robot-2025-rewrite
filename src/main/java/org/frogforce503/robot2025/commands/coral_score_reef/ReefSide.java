package org.frogforce503.robot2025.commands.coral_score_reef;

import java.util.function.Supplier;

import org.frogforce503.robot2025.fields.FieldInfo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public enum ReefSide {
    RED_AB {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return () -> getReefTagAlignmentPose(field, 7);
        }
    },
    RED_CD {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return () -> getReefTagAlignmentPose(field, 8);
        }
    },
    RED_EF {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return () -> getReefTagAlignmentPose(field, 9);
        }
    },
    RED_GH {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return () -> getReefTagAlignmentPose(field, 10);
        }
    },
    RED_IJ {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return () -> getReefTagAlignmentPose(field, 11);
        }
    },
    RED_KL {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return () -> getReefTagAlignmentPose(field, 6);
        }
    },
    BLUE_AB {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return () -> getReefTagAlignmentPose(field, 18);
        }
    },
    BLUE_CD {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return () -> getReefTagAlignmentPose(field, 17);
        }
    },
    BLUE_EF {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return () -> getReefTagAlignmentPose(field, 22);
        }
    },
    BLUE_GH {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return () -> getReefTagAlignmentPose(field, 21);
        }
    },
    BLUE_IJ {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return () -> getReefTagAlignmentPose(field, 20);
        }
    },
    BLUE_KL {
        @Override
        public Supplier<Pose2d> getTarget(FieldInfo field) {
            return () -> getReefTagAlignmentPose(field, 19);
        }
    };
    
    public abstract Supplier<Pose2d> getTarget(FieldInfo field);

    protected Pose2d getReefTagAlignmentPose(FieldInfo field, int tagId) {
        Pose2d tagPose = field.getTagById(tagId);

        return new Pose2d(
            tagPose
                .getTranslation()
                .plus(
                    new Translation2d(Units.inchesToMeters(19.0), tagPose.getRotation())),
            tagPose
                .getRotation()
                .plus(Rotation2d.kPi));
    }
}
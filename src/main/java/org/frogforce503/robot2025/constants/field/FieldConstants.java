package org.frogforce503.robot2025.constants.field;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    private static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final double fieldLength = aprilTagFieldLayout.getFieldLength();
    public static final double fieldWidth = aprilTagFieldLayout.getFieldWidth();

    public static class Processor {
        public static final Pose2d blue =
            new Pose2d(
                aprilTagFieldLayout.getTagPose(16).get().getX(),
                0,
                Rotation2d.fromDegrees(90));
                
        public static final Pose2d red =
            new Pose2d(
                aprilTagFieldLayout.getTagPose(3).get().getX(),
                fieldWidth,
                Rotation2d.fromDegrees(-90));
    }

    public static class CoralStation {
        public static final double stationLength = Units.inchesToMeters(79.750);
        
        public static final Pose2d blueLeft = aprilTagFieldLayout.getTagPose(13).get().toPose2d();
        public static final Pose2d blueRight = aprilTagFieldLayout.getTagPose(12).get().toPose2d();

        public static final Pose2d redLeft = aprilTagFieldLayout.getTagPose(1).get().toPose2d();
        public static final Pose2d redRight = aprilTagFieldLayout.getTagPose(2).get().toPose2d();
    }

    // public static class Reef {
    //     public static final double faceLength = Units.inchesToMeters(36.792600);
    //     public static final Translation2d center =
    //         new Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.0);
    //     public static final double faceToZoneLine =
    //         Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    //     public static final Pose2d[] centerFaces =
    //         new Pose2d[6]; // Starting facing the driver station in clockwise order
    //     public static final List<Map<ReefLevel, Pose3d>> branchPositions =
    //         new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise
    //     public static final List<Map<ReefLevel, Pose2d>> branchPositions2d = new ArrayList<>();

    //     static {
    //     // Initialize faces
    //     var aprilTagLayout = AprilTagLayoutType.OFFICIAL.getLayout();
    //     centerFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
    //     centerFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
    //     centerFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
    //     centerFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
    //     centerFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
    //     centerFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d();

    //     // Initialize branch positions
    //     for (int face = 0; face < 6; face++) {
    //         Map<ReefLevel, Pose3d> fillRight = new HashMap<>();
    //         Map<ReefLevel, Pose3d> fillLeft = new HashMap<>();
    //         Map<ReefLevel, Pose2d> fillRight2d = new HashMap<>();
    //         Map<ReefLevel, Pose2d> fillLeft2d = new HashMap<>();
    //         for (var level : ReefLevel.values()) {
    //         Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
    //         double adjustX = Units.inchesToMeters(30.738);
    //         double adjustY = Units.inchesToMeters(6.469);

    //         var rightBranchPose =
    //             new Pose3d(
    //                 new Translation3d(
    //                     poseDirection
    //                         .transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero))
    //                         .getX(),
    //                     poseDirection
    //                         .transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero))
    //                         .getY(),
    //                     level.height),
    //                 new Rotation3d(
    //                     0,
    //                     Units.degreesToRadians(level.pitch),
    //                     poseDirection.getRotation().getRadians()));
    //         var leftBranchPose =
    //             new Pose3d(
    //                 new Translation3d(
    //                     poseDirection
    //                         .transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero))
    //                         .getX(),
    //                     poseDirection
    //                         .transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero))
    //                         .getY(),
    //                     level.height),
    //                 new Rotation3d(
    //                     0,
    //                     Units.degreesToRadians(level.pitch),
    //                     poseDirection.getRotation().getRadians()));

    //         fillRight.put(level, rightBranchPose);
    //         fillLeft.put(level, leftBranchPose);
    //         fillRight2d.put(level, rightBranchPose.toPose2d());
    //         fillLeft2d.put(level, leftBranchPose.toPose2d());
    //         }
    //         branchPositions.add(fillRight);
    //         branchPositions.add(fillLeft);
    //         branchPositions2d.add(fillRight2d);
    //         branchPositions2d.add(fillLeft2d);
    //     }
    //     }
    // }

    // public static class StagingPositions {
    //     // Measured from the center of the ice cream
    //     public static final double separation = Units.inchesToMeters(72.0);
    //     public static final Translation2d[] iceCreams = new Translation2d[3];

    //     static {
    //     for (int i = 0; i < 3; i++) {
    //         iceCreams[i] =
    //             new Translation2d(
    //                 Units.inchesToMeters(48), fieldWidth / 2.0 - separation + separation * i);
    //     }
    //     }
    // }
}

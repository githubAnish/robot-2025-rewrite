package org.frogforce503.robot2025.constants.field;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.lib.util.FieldConstantsUtil;
import org.frogforce503.robot2025.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static final AprilTagFieldLayout aprilTagFieldLayout = Constants.fieldVenue.getAprilTagFieldLayout();

    public static final double fieldLength = aprilTagFieldLayout.getFieldLength();
    public static final double fieldWidth = aprilTagFieldLayout.getFieldWidth();

    public static Pose2d getTagPose2d(int tagId) {
        return
            aprilTagFieldLayout
                .getTagPose(tagId)
                .orElseThrow(() -> new IllegalArgumentException("No tag with ID " + tagId + " found in layout" + ErrorUtil.attachJavaClassName(FieldConstants.class)))
                .toPose2d();
    }

    public static class Lines {
        public static final double blueInitLineX;
        public static final double redInitLineX;

        static {
            final double BlueInitLineToLeftCage = FieldConstantsUtil.getFieldValueMeters("BlueInitLineToLeftCage");
            blueInitLineX = fieldLength / 2 - BlueInitLineToLeftCage;

            final double RedInitLineToLeftCage = FieldConstantsUtil.getFieldValueMeters("RedInitLineToLeftCage");
            redInitLineX = fieldLength / 2 + RedInitLineToLeftCage;
        }
    }

    public static class Processor {
        public static final Pose2d blue = new Pose2d(aprilTagFieldLayout.getTagPose(16).get().getX(), 0, Rotation2d.fromDegrees(90));
        public static final Pose2d red = new Pose2d(aprilTagFieldLayout.getTagPose(3).get().getX(), fieldWidth, Rotation2d.fromDegrees(-90));
    }

    public static class CoralStation {
        public static final double stationLength = Units.inchesToMeters(79.750);

        public static final Pose2d blueLeft = aprilTagFieldLayout.getTagPose(13).get().toPose2d();
        public static final Pose2d blueRight = aprilTagFieldLayout.getTagPose(12).get().toPose2d();

        public static final Pose2d redLeft = aprilTagFieldLayout.getTagPose(1).get().toPose2d();
        public static final Pose2d redRight = aprilTagFieldLayout.getTagPose(2).get().toPose2d();
    }

    public static class Reef {
        // Blue reef
        public static final Pose2d[] blueFaceCenters =
            new Pose2d[] {
                aprilTagFieldLayout.getTagPose(18).get().toPose2d(), // blueCenterAB
                aprilTagFieldLayout.getTagPose(17).get().toPose2d(), // blueCenterCD 
                aprilTagFieldLayout.getTagPose(22).get().toPose2d(), // blueCenterEF 
                aprilTagFieldLayout.getTagPose(21).get().toPose2d(), // blueCenterGH 
                aprilTagFieldLayout.getTagPose(20).get().toPose2d(), // blueCenterIJ 
                aprilTagFieldLayout.getTagPose(19).get().toPose2d()  // blueCenterKL
            };

        public static final Translation2d blueReefCenter =
            blueFaceCenters[0]
                .getTranslation()
                .interpolate(blueFaceCenters[3].getTranslation(), 0.5); // middle of AB and GH faces

        public static final Pose2d[] blueLeftBranches = new Pose2d[6];
        public static final Pose2d[] blueRightBranches = new Pose2d[6];

        // Red reef
        public static final Pose2d[] redFaceCenters =
            new Pose2d[] {
                aprilTagFieldLayout.getTagPose(7).get().toPose2d(), // redCenterAB
                aprilTagFieldLayout.getTagPose(8).get().toPose2d(), // redCenterCD 
                aprilTagFieldLayout.getTagPose(9).get().toPose2d(), // redCenterEF 
                aprilTagFieldLayout.getTagPose(10).get().toPose2d(), // redCenterGH 
                aprilTagFieldLayout.getTagPose(11).get().toPose2d(), // redCenterIJ 
                aprilTagFieldLayout.getTagPose(6).get().toPose2d()  // redCenterKL
            };

        public static final Translation2d redReefCenter =
            redFaceCenters[0]
                .getTranslation()
                .interpolate(redFaceCenters[3].getTranslation(), 0.5); // middle of AB and GH faces

        public static final Pose2d[] redLeftBranches = new Pose2d[6];
        public static final Pose2d[] redRightBranches = new Pose2d[6];

        static {
            final double adjustX = Units.inchesToMeters(2.007); // measured distance from face AprilTag X to branch X
            final double adjustY = Units.inchesToMeters(6.469); // measured distance from face AprilTag Y to branch Y 

            // Initialize blue branches
            for (int i = 0; i < 6; i++) {
                Pose2d faceCenter = Reef.blueFaceCenters[i];

                Pose2d leftBranch = faceCenter.plus(GeomUtil.toTransform2d(adjustX, -adjustY));
                Pose2d rightBranch = faceCenter.plus(GeomUtil.toTransform2d(adjustX, adjustY));

                blueLeftBranches[i] = leftBranch;
                blueRightBranches[i] = rightBranch;
            }

            // Initialize red branches
            for (int i = 0; i < 6; i++) {
                Pose2d faceCenter = Reef.redFaceCenters[i];

                Pose2d leftBranch = faceCenter.plus(GeomUtil.toTransform2d(adjustX, -adjustY));
                Pose2d rightBranch = faceCenter.plus(GeomUtil.toTransform2d(adjustX, adjustY));

                redLeftBranches[i] = leftBranch;
                redRightBranches[i] = rightBranch;
            }
        }
    }

    public static class IceCream {
        public static final Translation2d blueLeft;
        public static final Translation2d blueCenter;
        public static final Translation2d blueRight;

        public static final Translation2d redLeft;
        public static final Translation2d redCenter;
        public static final Translation2d redRight;

        static {
            final double centerIcecreamToReefDist = Units.inchesToMeters(95.25);
            final double iceCreamSeparationDist = Units.inchesToMeters(72);

            // Initialize blue ice creams
            blueCenter =
                Reef.blueFaceCenters[0]
                    .getTranslation()
                    .plus(new Translation2d(-centerIcecreamToReefDist, 0));
            blueLeft = blueCenter.plus(new Translation2d(0, iceCreamSeparationDist));
            blueRight = blueCenter.plus(new Translation2d(0, -iceCreamSeparationDist));

            // Initialize red ice creams
            redCenter =
                Reef.redFaceCenters[0]
                    .getTranslation()
                    .plus(new Translation2d(centerIcecreamToReefDist, 0));
            redLeft = redCenter.plus(new Translation2d(0, -iceCreamSeparationDist));
            redRight = redCenter.plus(new Translation2d(0, iceCreamSeparationDist));
        }
    }

    public static class Cage {
        public static final Translation2d blueLeft;
        public static final Translation2d blueCenter;
        public static final Translation2d blueRight;

        public static final Translation2d redLeft;
        public static final Translation2d redCenter;
        public static final Translation2d redRight;

        static {
            final double cageSeparationDist = Units.inchesToMeters(43);
            final double leftCageToWallDist = Units.inchesToMeters(31);

            // Initialize blue cages
            final double BlueInitLineToLeftCage = FieldConstantsUtil.getFieldValueMeters("BlueInitLineToLeftCage");

            blueLeft = new Translation2d(Lines.blueInitLineX + BlueInitLineToLeftCage, fieldWidth - leftCageToWallDist);
            blueCenter = blueLeft.plus(new Translation2d(0, -cageSeparationDist));
            blueRight = blueCenter.plus(new Translation2d(0, -cageSeparationDist));

            // Initialize red cages
            final double RedInitLineToLeftCage = FieldConstantsUtil.getFieldValueMeters("RedInitLineToLeftCage");

            redLeft = new Translation2d(Lines.redInitLineX - RedInitLineToLeftCage, leftCageToWallDist);
            redCenter = redLeft.plus(new Translation2d(0, cageSeparationDist));
            redRight = redCenter.plus(new Translation2d(0, cageSeparationDist));
        }
    }
}

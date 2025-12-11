package org.frogforce503.lib.util;

import org.frogforce503.robot2025.FieldInfo;
import org.frogforce503.robot2025.constants.field.FieldConstants;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public final class ProximityUtil {
    private ProximityUtil() {}

    private static double getDistanceBetweenPoses(Pose2d first, Pose2d second) {
        return first.minus(second).getTranslation().getNorm();
    }

    public static double getDistanceFromPose(Drive drive, Pose2d target) {
        return getDistanceBetweenPoses(drive.getCurrentPose(), target);
    }

    public static Pose2d getClosestPose(Drive drive, Pose2d... options) {
        if (options.length == 0) {
            return null;
        }

        Pose2d robotPose = drive.getCurrentPose();

        Pose2d closestPose = options[0];
        double closestDist = getDistanceBetweenPoses(closestPose, robotPose);

        for (Pose2d pose : options) {
            double dist = getDistanceBetweenPoses(pose, robotPose);

            if (dist < closestDist) {
                closestDist = dist;
                closestPose = pose;
            }
        }

        return closestPose;
    }

    public static Pose2d getClosestProcessor(Drive drive) {
        return
            getClosestPose(
                drive,
                FieldConstants.Processor.blue,
                FieldConstants.Processor.red);
    }

    public static Pose2d getClosestStation(Drive drive) {
        return
            getClosestPose(
                drive,
                FieldConstants.CoralStation.blueLeft,
                FieldConstants.CoralStation.blueRight,
                FieldConstants.CoralStation.redLeft,
                FieldConstants.CoralStation.redRight);
    }

    public static Pose2d getClosestReefSide(Drive drive) {
        return
            getClosestPose(
                drive,
                FieldInfo.isRed() ? FieldConstants.Reef.redFaceCenters : FieldConstants.Reef.blueFaceCenters);
    }

    public static double getClosestBargeX(Drive drive) {
        return FieldInfo.isRed()
            ? FieldConstants.Lines.redInitLineX
            : FieldConstants.Lines.redInitLineX;
    }
}

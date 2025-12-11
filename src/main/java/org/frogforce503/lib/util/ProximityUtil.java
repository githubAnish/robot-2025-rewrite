package org.frogforce503.lib.util;

import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public final class ProximityUtil {
    private ProximityUtil() {}

    private static double getDistanceBetweenPoses(Pose2d first, Pose2d second) {
        return first.minus(second).getTranslation().getNorm();
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
}

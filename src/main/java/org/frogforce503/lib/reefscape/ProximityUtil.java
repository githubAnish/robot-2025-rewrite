package org.frogforce503.lib.reefscape;

import java.util.Arrays;

import org.frogforce503.robot.FieldInfo;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public final class ProximityUtil {
    private ProximityUtil() {}

    // General-purpose proximity methods
    private static double getDistanceBetweenPoses(Pose2d first, Pose2d second) {
        return first.minus(second).getTranslation().getNorm();
    }

    public static double getDistanceFromPose(Drive drive, Pose2d target) {
        return getDistanceBetweenPoses(drive.getPose(), target);
    }

    public static Pose2d getClosestPose(Drive drive, Pose2d... options) {
        if (options.length == 0) {
            return null;
        }

        return drive.getPose().nearest(Arrays.asList(options));
    }

    // Reefscape-specific proximity methods
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
                FieldInfo.isRed()
                    ? FieldConstants.Reef.redFaceCenters
                    : FieldConstants.Reef.blueFaceCenters);
    }

    public static Pose2d getClosestLeftBranch(Drive drive) {
        return
            getClosestPose(
                drive,
                FieldInfo.isRed()
                    ? FieldConstants.Reef.redLeftBranches
                    : FieldConstants.Reef.blueLeftBranches);
    }

    public static Pose2d getClosestRightBranch(Drive drive) {
        return
            getClosestPose(
                drive,
                FieldInfo.isRed()
                    ? FieldConstants.Reef.redRightBranches
                    : FieldConstants.Reef.blueRightBranches);
    }

    public static double getClosestBargeX(Drive drive) {
        return FieldInfo.isRed()
            ? FieldConstants.Lines.redInitLineX
            : FieldConstants.Lines.blueInitLineX;
    }
}

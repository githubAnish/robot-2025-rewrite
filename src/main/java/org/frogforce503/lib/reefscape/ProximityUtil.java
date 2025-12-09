package org.frogforce503.lib.reefscape;

import java.util.Arrays;
import java.util.function.Function;
import java.util.function.Supplier;

import org.frogforce503.robot2025.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public final class ProximityUtil {
    private ProximityUtil() {}

    private static double distanceBetweenPoses(Pose2d first, Pose2d second) {
        return
            first
                .minus(second)
                .getTranslation()
                .getNorm();
    }

    private static int poseDistanceComparator(Pose2d target, Pose2d first, Pose2d second) {
        return
            Double.compare(
                distanceBetweenPoses(first, target),
                distanceBetweenPoses(second, target));
    }

    public static <T> T closestTo(Supplier<Pose2d> target, Function<T, Pose2d> poseExtractor, T... options) {
        return
            Arrays
                .stream(options)
                .min(
                    (option1, option2) ->
                        poseDistanceComparator(
                            target.get(),
                            poseExtractor.apply(option1),
                            poseExtractor.apply(option2)))
                .orElse(options[0]); // Return the first option if none are closest
    }

    public static <T> T chassisClosestTo(Drive drive, Function<T, Pose2d> poseExtractor, T... options) {
        return closestTo(drive::getCurrentPose, poseExtractor, options);
    }

    private static Pose2d chassisClosestTo(Drive drive, Pose2d... options) {
        return chassisClosestTo(drive, Function.identity(), options);
    }

    public static ReefSide getClosestReefSide(Drive drive, FieldInfo field) {
        return
            chassisClosestTo(
                drive,
                side -> side.getTarget(field).get(),
                ReefSide.values());
    }

    public static Station getClosestStation(Drive drive, FieldInfo field) {
        return
            chassisClosestTo(
                drive,
                station -> station.getTarget(field).get(),
                Station.values());
    }

    public static Processor getClosestProcessor(Drive drive, FieldInfo field) {
        return
            chassisClosestTo(
                drive,
                processor -> processor.getTarget(field).get(),
                Processor.values());
    }

    public static Barge getClosestBarge(Drive drive, FieldInfo field) {
        return
            chassisClosestTo(
                drive,
                barge -> barge.getTarget(drive, field).get(),
                Barge.values());
    }
}
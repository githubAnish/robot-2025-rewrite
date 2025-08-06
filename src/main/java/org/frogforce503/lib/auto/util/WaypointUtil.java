package org.frogforce503.lib.auto.util;

import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

import org.frogforce503.lib.planning.planned_path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class WaypointUtil {
    public static Pose2d flipHeading(Pose2d existing) {
        return new Pose2d(existing.getTranslation(), existing.getRotation().unaryMinus());
    }

    private static List<Waypoint> changeAllWaypoints(List<Waypoint> input, Function<Waypoint, Waypoint> changeFunction) {
        return
            input
                .stream()
                .map(changeFunction)
                .collect(Collectors.toList());
    }

    public static List<Waypoint> shift(List<Waypoint> input, Translation2d delta) {
        return changeAllWaypoints(input, waypoint -> waypoint.plus(delta));
    }

    public static List<Waypoint> withHeading(List<Waypoint> input, Rotation2d heading) {
        return changeAllWaypoints(input, waypoint -> waypoint.withHolonomicRotation(heading));
    }
}
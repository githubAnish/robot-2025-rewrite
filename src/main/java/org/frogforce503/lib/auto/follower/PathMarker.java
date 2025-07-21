package org.frogforce503.lib.auto.follower;

import org.frogforce503.lib.trajectory.PlannedPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

/**
 * @param center Center of the path marker.
 * @param rangeMeters Range of the path marker, in meters.
 * @param onEnter {@link Runnable} to execute when the robot enters the marker.
 * @param periodic {@link Runnable} to execute periodically while the robot is in the marker.
 * @param onExit {@link Runnable} to execute when the robot exits the marker.
 */
public record PathMarker(Translation2d center, double rangeMeters, Runnable onEnter, Runnable periodic, Runnable onExit) {
    public PathMarker(Translation2d center, double rangeMeters, Runnable onEnter) {
        this(center, rangeMeters, onEnter, () -> {}, () -> {});
    }

    public PathMarker(Trajectory.State state, double rangeMeters, Runnable onEnter, Runnable periodic, Runnable onExit) {
        this(state.poseMeters.getTranslation(), rangeMeters, onEnter, periodic, onExit);
    }

    public PathMarker(Trajectory.State state, double rangeMeters, Runnable onEnter) {
        this(state.poseMeters.getTranslation(), rangeMeters, onEnter, () -> {}, () -> {});
    }

    public PathMarker(PlannedPath path, double percentageOfPathComplete, Runnable onEnter, Runnable periodic, Runnable onExit) {
        this(
            path
                .sample(path.getDriveTrajectory().getTotalTimeSeconds() * percentageOfPathComplete)
                .poseMeters()
                .getTranslation(),
            0.5,
            onEnter,
            periodic,
            onExit);
    }

    public PathMarker(PlannedPath path, Runnable onEnter) {
        this(
            path,
            0.5,
            onEnter,
            () -> {},
            () -> {});
    }
}
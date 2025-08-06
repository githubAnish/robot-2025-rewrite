package org.frogforce503.lib.auto.route;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import lombok.Getter;

@SuppressWarnings("unchecked")
public abstract class BaseRoute<T> {
    @Getter List<T> paths;

    public BaseRoute() {
        paths = new ArrayList<>();
    }

    public abstract List<Pose2d> getPoses();
    public abstract Pose2d getInitialPose();

    public <X extends BaseRoute<T>> X addPaths(T... paths) {
        for (T path : paths) {
            this.paths.add(path);
        }
        return (X) this;
    }

    public T getFirstPath() {
        return this.getPaths().get(0);
    }

    public Pose2d getStartingPose(Supplier<Pose2d> override) {
        if (this.paths.size() == 0) {
            return Pose2d.kZero;
        }

        // Create an Optional of the first path to check if it exists
        var firstPath = Optional.ofNullable(getFirstPath());

        return
            firstPath.isPresent()
                ? getInitialPose()
                : override.get();
    }
}
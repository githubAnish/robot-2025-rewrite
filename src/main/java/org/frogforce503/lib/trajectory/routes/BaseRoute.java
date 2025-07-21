package org.frogforce503.lib.trajectory.routes;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import lombok.Getter;

@SuppressWarnings("unchecked")
public abstract class BaseRoute<T> {
    @Getter List<T> paths;
    boolean knownStart = false;

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
    
    public <X extends BaseRoute<T>> X withKnownStart() {
        this.knownStart = true;
        return (X) this;
    }

    public boolean hasKnownStart() {
        return this.knownStart;
    }

    public T getFirstPath() {
        return this.getPaths().get(0);
    }

    public Pose2d getStartingPose(Supplier<Pose2d> override) {
        if (this.paths.size() == 0)
            return new Pose2d();

        var p = getFirstPath();
        return (p != null ? getInitialPose() : override.get());
    }
}
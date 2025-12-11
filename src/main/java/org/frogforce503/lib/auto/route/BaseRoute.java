package org.frogforce503.lib.auto.route;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;

public interface BaseRoute {
    public List<Pose2d> getPoses();
    public Optional<Pose2d> getInitialPose();
}
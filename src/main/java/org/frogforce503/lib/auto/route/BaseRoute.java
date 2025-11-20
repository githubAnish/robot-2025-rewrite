package org.frogforce503.lib.auto.route;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;

public interface BaseRoute {
    public List<Pose2d> getPoses();
    public Pose2d getInitialPose(Supplier<Pose2d> overridePoseSupplier);
}
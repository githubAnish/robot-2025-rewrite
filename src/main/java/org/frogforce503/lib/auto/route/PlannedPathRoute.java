package org.frogforce503.lib.auto.route;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.frogforce503.lib.planning.planned_path.PlannedPath;

import edu.wpi.first.math.geometry.Pose2d;

public class PlannedPathRoute implements BaseRoute {
    private final List<PlannedPath> paths = new ArrayList<>();

    public PlannedPathRoute(PlannedPath... paths) {
        for (PlannedPath path : paths) {
            this.paths.add(path);
        }
    }

    @Override
    public Pose2d getInitialPose(Supplier<Pose2d> overridePoseSupplier) {
        Optional<Pose2d> initialPose =
            Optional.ofNullable(
                paths
                    .get(0)
                    .getInitialHolonomicPose());

        return initialPose.orElseGet(overridePoseSupplier);
    }

    @Override
    public List<Pose2d> getPoses() {
        return
            paths
                .stream()
                .flatMap(path -> path.getDriveTrajectory().getStates().stream())
                .map(state -> state.poseMeters)
                .collect(Collectors.toList());
    }
}

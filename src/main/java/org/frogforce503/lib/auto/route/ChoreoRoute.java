package org.frogforce503.lib.auto.route;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.frogforce503.robot2025.Constants;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;

public class ChoreoRoute implements BaseRoute {
    private final List<AutoTrajectory> paths = new ArrayList<>();

    public ChoreoRoute(AutoTrajectory... paths) {
        for (AutoTrajectory path : paths) {
            this.paths.add(path);
        }
    }

    @Override
    public List<Pose2d> getPoses() {
        return
            paths
                .stream()
                .flatMap(traj -> Arrays.stream(traj.getRawTrajectory().getPoses()))
                .collect(Collectors.toList());
    }

    @Override
    public Pose2d getInitialPose(Supplier<Pose2d> overridePoseSupplier) {
        return
            paths
                .get(0)
                .getRawTrajectory()
                .getInitialPose(Constants.useAllianceFlipping)
                .orElseGet(overridePoseSupplier);
    }
}

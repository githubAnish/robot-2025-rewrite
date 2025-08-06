package org.frogforce503.lib.auto.route;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;

public class ChoreoRoute extends BaseRoute<AutoTrajectory> {
    public ChoreoRoute(AutoTrajectory... paths) {
        Arrays
            .stream(paths)
            .forEach(
                path ->
                    this.paths.add(path));
    }

    public ChoreoRoute(List<AutoTrajectory> paths) {
        this(paths.toArray(AutoTrajectory[]::new));
    }

    @Override
    public Pose2d getInitialPose() {
        return
            getFirstPath()
                .getRawTrajectory()
                .getInitialPose(false)
                .orElseGet(Pose2d::new);
    }

    @Override
    public List<Pose2d> getPoses() {
        return
            this.paths
                .stream()
                .flatMap(
                    traj ->
                        Arrays.stream(traj.getRawTrajectory().getPoses()))
                .collect(Collectors.toList());
    }
}

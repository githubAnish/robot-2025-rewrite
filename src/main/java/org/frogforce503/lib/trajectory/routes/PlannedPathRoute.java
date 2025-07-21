package org.frogforce503.lib.trajectory.routes;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.frogforce503.lib.trajectory.PlannedPath;

import edu.wpi.first.math.geometry.Pose2d;

public class PlannedPathRoute extends BaseRoute<PlannedPath> {
    public PlannedPathRoute(List<PlannedPath> paths) {
        this.paths.addAll(paths);
    }

    public PlannedPathRoute(PlannedPath... paths) {
        this(Arrays.asList(paths));
    }

    @Override
    public Pose2d getInitialPose() {
        return getFirstPath().getInitialHolonomicPose();
    }

    @Override
    public List<Pose2d> getPoses() {
        List<Pose2d> poses = new ArrayList<>();

        this.paths
            .stream()
            .flatMap(
                path ->
                    path.getDriveTrajectory().getStates().stream())
            .forEach(
                state ->
                    poses.add(state.poseMeters));

        return poses;
    }

    public PlannedPathRoute addTrees(Tree... trees) {
        for (Tree tree : trees) {
            if (tree.trunk != null)
                this.paths.add(tree.trunk);
            
            this.paths.addAll(Arrays.asList(tree.branches));
        }
        
        return this;
    }

    public static class Tree {
        public PlannedPath trunk;
        public PlannedPath[] branches;
        public PlannedPath fallbackPath;

        public Tree withTrunk(PlannedPath trunk) {
            this.trunk = trunk;
            return this;
        }

        public Tree withBranches(PlannedPath... branches) {
            this.branches = branches;
            return this;
        }

        public Tree withFallbackPath(PlannedPath path) {
            this.fallbackPath = path;
            return this;
        }
    }
}

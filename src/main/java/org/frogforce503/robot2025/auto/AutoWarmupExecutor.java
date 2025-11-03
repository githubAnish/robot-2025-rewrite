package org.frogforce503.robot2025.auto;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.frogforce503.lib.auto.builder.PlannedPathGenerator;
import org.frogforce503.lib.planning.planned_path.PlannedPath;
import org.frogforce503.lib.planning.planned_path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class AutoWarmupExecutor {
    private final AutoChooser autoChooser;

    public AutoWarmupExecutor(AutoChooser autoChooser) {
        this.autoChooser = autoChooser;
    }

    public void execute() {
        warmupPlannedPathGenerator();
        warmupAutos();
    }

    /** Warmups the {@link PlannedPathGenerator} to speed up {@link PlannedPath} generation. */
    private void warmupPlannedPathGenerator() {
        // Random values inserted in for path constraints & waypoints
        PlannedPathGenerator
            .generate(
                Units.inchesToMeters(13),
                Units.inchesToMeters(19),
                3.0,
                0.7,
                Waypoint.fromHolonomicPose(new Pose2d(0, 0, new Rotation2d(Math.PI/4))),
                Waypoint.fromHolonomicPose(new Pose2d(2.5, 2.5, new Rotation2d(-Math.PI/3))),
                Waypoint.fromHolonomicPose(new Pose2d(10, 7.5, new Rotation2d(Math.PI))));
    }

    private void warmupAutos() {
        List<AutoMode> autos =
            autoChooser
                .getAutoMap()
                .getAutos()
                .stream()
                .map(Supplier::get)
                .collect(Collectors.toList());

        for (AutoMode auto : autos) {
            auto.routine().ignoringDisable(true).schedule();
        }
    }

    private void printWarmupTime(Runnable action) {
        long startTime = System.nanoTime();
        
        action.run();

        long endTime = System.nanoTime();
        
        System.out.println("Warmup took " + (endTime - startTime)/1e9 + " s");
    }
}
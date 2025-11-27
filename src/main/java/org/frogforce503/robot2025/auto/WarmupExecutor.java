package org.frogforce503.robot2025.auto;

import org.frogforce503.lib.auto.builder.PlannedPathGenerator;
import org.frogforce503.lib.planning.planned_path.PlannedPath;
import org.frogforce503.lib.planning.planned_path.Waypoint;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.drive.DriveConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class WarmupExecutor {
    private final Drive drive;
    private final AutoChooser autoChooser;

    public WarmupExecutor(Drive drive, AutoChooser autoChooser) {
        this.drive = drive;
        this.autoChooser = autoChooser;
    }

    private void printWarmupTime(Runnable action) {
        long startTime = System.nanoTime();
        action.run();
        long endTime = System.nanoTime();
        System.out.println("Warmup took " + (endTime - startTime)/1e9 + " s");
    }

    public void disabledInit() {
        // NetworkTableInstance.getDefault().flush();
        // System.gc();
    }

    public void disabledPeriodic() {
        warmupPlannedPathGenerator();
        warmupPaths();
        warmupDrive();
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

    private void warmupPaths() {
        
    }

    private void warmupDrive() {
        // Warmup path follower
        DriveConstants.pathFollower.calculate(
            drive.getCurrentPose(),
            Pose2d.kZero,
            0.1,
            0.1,
            0.1);
    }
}
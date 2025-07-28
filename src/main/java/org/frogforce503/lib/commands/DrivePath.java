package org.frogforce503.lib.commands;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import org.frogforce503.lib.auto.builder.PlannedPathBuilder;
import org.frogforce503.lib.auto.trajectory.path.PlannedPath;
import org.frogforce503.lib.auto.trajectory.path.Waypoint;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Follows a runtime-generated {@link PlannedPath}. */
public class DrivePath extends Command {
    private final Drive drive;
    private final FieldInfo field;
    
    private final PlannedPathBuilder pathBuilder;

    private final Supplier<Pose2d> robotPose;
    private final Supplier<Pose2d> target;
    private final List<Waypoint> waypointsBetween;

    private final PathLimits limits;

    private FollowPlannedPath pathFollowingCommand;

    @SuppressWarnings("unchecked")
    public DrivePath(Drive drive, FieldInfo field, PathLimits limits, Supplier<Pose2d> robotPose, Supplier<Pose2d> target, Supplier<Pose2d>... posesBetween) {
        this.drive = drive;
        this.field = field;

        this.pathBuilder = new PlannedPathBuilder();

        this.robotPose = robotPose;
        this.target = target;
        this.limits = limits;

        this.waypointsBetween =
            Arrays
                .stream(posesBetween)
                .map(poseSupplier -> Waypoint.fromHolonomicPose(poseSupplier.get()))
                .toList();

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        this.waypointsBetween.add(0, Waypoint.fromHolonomicPose(robotPose.get()));
        this.waypointsBetween.add(Waypoint.fromHolonomicPose(target.get()));

        pathFollowingCommand =
            new FollowPlannedPath(
                drive,
                field,
                pathBuilder
                    .generate(
                        limits.maxVelocity(),
                        limits.maxAcceleration(),
                        0.0,
                        0.0,
                        waypointsBetween));

        pathFollowingCommand.schedule();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return pathFollowingCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
    
    public record PathLimits(double maxVelocity, double maxAcceleration) {
        // Initialize path constraints with default values.
        public PathLimits() {
            this(3.0, 6.0);
        }
    }
}

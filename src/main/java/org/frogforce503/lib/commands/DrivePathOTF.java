package org.frogforce503.lib.commands;

import java.util.List;
import java.util.function.Supplier;

import org.frogforce503.lib.auto.builder.PlannedPathBuilder;
import org.frogforce503.lib.planning.planned_path.PlannedPath;
import org.frogforce503.lib.planning.planned_path.Waypoint;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;

/** Follows a runtime-generated {@link PlannedPath}. */
public class DrivePathOTF extends Command {
    private final Drive drive;
    private final FieldInfo field;
    private final PlannedPathBuilder pathBuilder;

    private final Supplier<Pose2d> robotPose;
    private final Supplier<Pose2d> target;
    private final Constraints constraints;

    private FollowPlannedPath pathFollowingCommand;

    public DrivePathOTF(
        Drive drive,
        FieldInfo field,
        Constraints constraints,
        Supplier<Pose2d> robotPose,
        Supplier<Pose2d> target
    ) {
        this.drive = drive;
        this.field = field;

        this.pathBuilder = new PlannedPathBuilder();

        this.robotPose = robotPose;
        this.target = target;
        this.constraints = constraints;

        addRequirements(drive);
    }

    public DrivePathOTF(
        Drive drive,
        FieldInfo field,
        Supplier<Pose2d> robotPose,
        Supplier<Pose2d> target
    ) {
        this(
            drive,
            field,
            new Constraints(3.0, 6.0),
            robotPose,
            target);
    }

    @Override
    public void initialize() {
        ChassisSpeeds currentVelocity = drive.getCurrentVelocity();
            
        final double initialRobotTranslationalVelocity =
            Math.hypot(
                currentVelocity.vxMetersPerSecond,
                currentVelocity.vyMetersPerSecond);

        pathFollowingCommand =
            new FollowPlannedPath(
                drive,
                field,
                pathBuilder
                    .generate(
                        constraints.maxVelocity,
                        constraints.maxAcceleration,
                        initialRobotTranslationalVelocity,
                        0.0,
                        List.of(
                            Waypoint.fromHolonomicPose(robotPose.get()),
                            Waypoint.fromHolonomicPose(target.get()))));

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
}

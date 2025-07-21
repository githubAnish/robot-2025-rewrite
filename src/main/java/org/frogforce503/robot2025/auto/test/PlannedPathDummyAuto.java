package org.frogforce503.robot2025.auto.test;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.lib.trajectory.routes.PlannedPathRoute;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Dummy auto using a PlannedPath. This auto is purely to show the format of how to create a auto using a PlannedPath, so don't try to run this auto, as it will crash. */
public class PlannedPathDummyAuto extends AutoMode {
    private final PlannedPath trajectory;

    public PlannedPathDummyAuto(Drive drive, FieldInfo field) {
        super(drive, field);
        
        SwervePathBuilder pathBuilder = new SwervePathBuilder();

        this.trajectory =
            pathBuilder.generate(0.0, 0.0, 0.0, 0.0,
                new Waypoint(),
                new Waypoint(new Translation2d(3.0, 3.0), null, Rotation2d.kPi));
    }

    @Override
    public Command routine() {
        return
            Commands.sequence(
                drive(trajectory)
            );
    }

    @Override
    public PlannedPathRoute getRoute() {
        return new PlannedPathRoute(trajectory);
    }
}

package org.frogforce503.robot2025.auto.test;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.auto.builder.PlannedPathBuilder;
import org.frogforce503.lib.auto.route.PlannedPathRoute;
import org.frogforce503.lib.planning.planned_path.PlannedPath;
import org.frogforce503.lib.planning.planned_path.Waypoint;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Dummy auto using a {@link PlannedPath}. This auto is purely to show the format of how to create a auto using a {@link PlannedPath}, so don't try to run this auto, as it will crash. */
public class PlannedPathDummyAuto extends AutoMode {
    private final PlannedPath trajectory;

    public PlannedPathDummyAuto(Drive drive, FieldInfo field, Superstructure superstructure) {
        super(drive, field, superstructure);
        
        PlannedPathBuilder pathBuilder = new PlannedPathBuilder();

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

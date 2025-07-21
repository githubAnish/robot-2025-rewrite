package org.frogforce503.robot2025.commands.algae_backup;

import java.util.function.Supplier;

import org.frogforce503.lib.commands.DriveToPose;
import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.robot2025.commands.coral_score_reef.ReefSide;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class BackupFromReefAlgae extends DriveToPose {
    public BackupFromReefAlgae(Drive drive, FieldInfo field, Supplier<Pose2d> robotPose, ReefSide side) {
        super(
            drive,
            field,
            robotPose,
            () ->
                side
                    .getTarget(field)
                    .get()
                    .plus(
                        GeomUtil.toTransform2d(Units.inchesToMeters(-4), 0.0)));
    }
}
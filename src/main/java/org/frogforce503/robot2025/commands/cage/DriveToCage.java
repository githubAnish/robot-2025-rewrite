package org.frogforce503.robot2025.commands.cage;

import java.util.function.Supplier;

import org.frogforce503.lib.commands.DriveToPose;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public class DriveToCage extends DriveToPose {
    public DriveToCage(Drive drive, FieldInfo field, Supplier<Pose2d> robotPose, Cage cage) {
        super(
            drive,
            field,
            robotPose,
            cage.getTarget(field));
    }
}
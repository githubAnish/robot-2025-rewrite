package org.frogforce503.robot2025.commands.algae_backup;

import java.util.function.Supplier;

import org.frogforce503.lib.util.ProximityUtil;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public class BackupFromClosestReefAlgae extends BackupFromReefAlgae {   
    public BackupFromClosestReefAlgae(Drive drive, FieldInfo field, Supplier<Pose2d> robotPose) {
        super(
            drive,
            field,
            robotPose,
            ProximityUtil.getClosestReefSide(drive, field));
    }

    /** Initializes the robot pose to the global pose of the robot. */
    public BackupFromClosestReefAlgae(Drive drive, FieldInfo field) {
        this(
            drive,
            field,
            drive::getCurrentPose);
    }
}
package org.frogforce503.robot2025.commands.algae_score_processor;

import java.util.function.Supplier;

import org.frogforce503.lib.util.ProximityUtil;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public class DriveToClosestProcessor extends DriveToProcessor {   
    public DriveToClosestProcessor(Drive drive, FieldInfo field, Supplier<Pose2d> robotPose) {
        super(
            drive,
            field,
            robotPose,
            ProximityUtil.getClosestProcessor(drive, field));
    }

    /** Initializes the robot pose to the global pose of the robot. */
    public DriveToClosestProcessor(Drive drive, FieldInfo field) {
        this(
            drive,
            field,
            drive::getCurrentPose);
    }
}
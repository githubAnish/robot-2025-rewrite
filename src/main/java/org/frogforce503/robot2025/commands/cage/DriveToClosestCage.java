package org.frogforce503.robot2025.commands.cage;

import java.util.function.Supplier;

import org.frogforce503.lib.util.ProximityService;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public class DriveToClosestCage extends DriveToCage {   
    public DriveToClosestCage(Drive drive, FieldInfo field, ProximityService proximityService, Supplier<Pose2d> robotPose) {
        super(
            drive,
            field,
            robotPose,
            proximityService.getClosestCage());
    }

    /** Initializes the robot pose to the global pose of the robot. */
    public DriveToClosestCage(Drive drive, FieldInfo field, ProximityService proximityService) {
        this(
            drive,
            field,
            proximityService,
            drive::getCurrentPose);
    }
}
package org.frogforce503.robot2025.commands.algae_intake_pluck;

import java.util.function.Supplier;

import org.frogforce503.lib.util.ProximityService;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public class DriveToClosestReefAlgae extends DriveToReefAlgae {   
    public DriveToClosestReefAlgae(Drive drive, FieldInfo field, ProximityService proximityService, Supplier<Pose2d> robotPose) {
        super(
            drive,
            field,
            robotPose,
            proximityService.getClosestReefSide());
    }

    /** Initializes the robot pose to the global pose of the robot. */
    public DriveToClosestReefAlgae(Drive drive, FieldInfo field, ProximityService proximityService) {
        this(
            drive,
            field,
            proximityService,
            drive::getCurrentPose);
    }
}
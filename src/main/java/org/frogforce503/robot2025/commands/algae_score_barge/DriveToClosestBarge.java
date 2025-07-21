package org.frogforce503.robot2025.commands.algae_score_barge;

import java.util.function.Supplier;

import org.frogforce503.lib.io.JoystickInputs;
import org.frogforce503.lib.util.ProximityService;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public class DriveToClosestBarge extends DriveToBarge {   
    public DriveToClosestBarge(Drive drive, FieldInfo field, ProximityService proximityService, JoystickInputs inputs, Supplier<Pose2d> robotPose) {
        super(
            drive,
            field,
            inputs,
            robotPose,
            proximityService.getClosestBarge());
    }

    /** Initializes the robot pose to the global pose of the robot. */
    public DriveToClosestBarge(Drive drive, FieldInfo field, ProximityService proximityService, JoystickInputs inputs) {
        this(
            drive,
            field,
            proximityService,
            inputs,
            drive::getCurrentPose);
    }
}
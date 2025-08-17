package org.frogforce503.robot2025.commands.coral_intake_station;

import java.util.function.Supplier;

import org.frogforce503.lib.io.JoystickInputs;
import org.frogforce503.lib.util.ProximityService;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public class DriveToClosestStation extends DriveToStation {   
    public DriveToClosestStation(Drive drive, FieldInfo field, ProximityService proximityService, JoystickInputs inputs, Supplier<Pose2d> robotPose) {
        super(
            drive,
            field,
            inputs,
            robotPose,
            proximityService.getClosestStation());
    }

    /** Initializes the robot pose to the global pose of the robot. */
    public DriveToClosestStation(Drive drive, FieldInfo field, ProximityService proximityService, JoystickInputs inputs) {
        this(
            drive,
            field,
            proximityService,
            inputs,
            drive::getCurrentPose);
    }
}
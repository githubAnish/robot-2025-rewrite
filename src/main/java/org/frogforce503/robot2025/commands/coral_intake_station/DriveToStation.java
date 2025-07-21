package org.frogforce503.robot2025.commands.coral_intake_station;

import java.util.function.Supplier;

import org.frogforce503.lib.commands.DriveToPose;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public class DriveToStation extends DriveToPose {
    public DriveToStation(Drive drive, FieldInfo field, Supplier<Pose2d> robotPose, Station station) {
        super(
            drive,
            field,
            robotPose,
            station.getTarget(field));
    }
}
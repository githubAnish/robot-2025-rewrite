package org.frogforce503.robot2025.commands.drive;

import java.util.function.Supplier;

import org.frogforce503.lib.reefscape.Processor;
import org.frogforce503.lib.util.ProximityUtil;
import org.frogforce503.robot2025.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public class DriveToProcessor extends DriveToPose {
    public DriveToProcessor(Drive drive, FieldInfo field, Supplier<Pose2d> robotPose, Processor processor) {
        super(
            drive,
            field,
            robotPose,
            processor.getTarget(field));
    }

    public DriveToProcessor(Drive drive, FieldInfo field, Supplier<Pose2d> robotPose) {
        this(
            drive,
            field,
            robotPose,
            ProximityUtil.getClosestProcessor(drive, field));
    }

    /** Initializes the robot pose to the global pose of the robot. */
    public DriveToProcessor(Drive drive, FieldInfo field) {
        this(
            drive,
            field,
            drive::getCurrentPose);
    }
}
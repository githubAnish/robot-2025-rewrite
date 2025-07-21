package org.frogforce503.robot2025.commands.algae_score_processor;

import java.util.function.Supplier;

import org.frogforce503.lib.commands.DriveToPose;
import org.frogforce503.robot2025.fields.FieldInfo;
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

    /** Initializes the robot pose to the global pose of the robot. */
    public DriveToProcessor(Drive drive, FieldInfo field, Processor processor) {
        this(
            drive,
            field,
            drive::getCurrentPose,
            processor);
    }
}
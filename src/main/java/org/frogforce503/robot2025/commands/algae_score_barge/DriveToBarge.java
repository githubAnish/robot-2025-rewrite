package org.frogforce503.robot2025.commands.algae_score_barge;

import java.util.function.Supplier;

import org.frogforce503.lib.commands.DriveToPose;
import org.frogforce503.lib.io.JoystickInputs;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public class DriveToBarge extends DriveToPose {
    public DriveToBarge(Drive drive, FieldInfo field, JoystickInputs inputs, Supplier<Pose2d> robotPose, Barge barge) {
        super(
            drive,
            field,
            robotPose,
            barge.getTarget(drive, field),
            inputs);
    }
}
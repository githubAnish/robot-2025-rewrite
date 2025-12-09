package org.frogforce503.lib.auto;

import org.frogforce503.lib.planning.planned_path.PlannedPath;
import org.frogforce503.robot2025.commands.drive.DriveToPose;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;

public final class AutoUtil {
    private AutoUtil() {}

    /**
     * Returns the current robot pose with a fallback for simulation.
     * Only required in autos that start with a {@link PlannedPath} trajectory or {@link DriveToPose} command.
     */
    public Pose2d setupInitialPose(Drive drive, Pose2d simulationFallback) {
        return RobotBase.isReal() ? drive.getCurrentPose() : simulationFallback;
    }
}

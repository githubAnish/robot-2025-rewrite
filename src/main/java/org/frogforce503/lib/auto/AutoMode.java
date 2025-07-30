package org.frogforce503.lib.auto;

import java.util.function.Supplier;

import org.frogforce503.lib.auto.trajectory.path.PlannedPath;
import org.frogforce503.lib.auto.trajectory.route.BaseRoute;
import org.frogforce503.lib.commands.DriveToPose;
import org.frogforce503.lib.commands.FollowPlannedPath;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure.Mode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;

public abstract class AutoMode {
    private final Drive drive;
    private final FieldInfo field;

    @Getter private final String name;

    public AutoMode(Drive drive, FieldInfo field) {
        this.drive = drive;
        this.field = field;

        this.name = this.getClass().getSimpleName();
    }

    public abstract Command routine();
    public abstract BaseRoute<?> getRoute();

    public Pose2d getStartingPose(Supplier<Pose2d> override) {
        return
            getRoute()
                .getStartingPose(override);
    }

    /**
     * Returns the current robot pose with a fallback for simulation.
     * Only required in autos that start with a {@link PlannedPath} trajectory or {@link DriveToPose} command.
     */
    public Pose2d setupPose(Pose2d simulationFallback) {
        return
            RobotBase.isReal()
                ? drive.getCurrentPose()
                : simulationFallback;
    }

    // Utilities
    public Command drive(PlannedPath path) {
        return new FollowPlannedPath(drive, field, path);
    }

    public Command setSuperstructureMode(Superstructure superstructure, Mode mode) {
        return
            Commands.runOnce(() -> superstructure.setCurrentMode(mode))
                .ignoringDisable(true);
    }
}
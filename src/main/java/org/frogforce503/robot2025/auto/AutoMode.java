package org.frogforce503.robot2025.auto;

import java.util.function.Supplier;

import org.frogforce503.lib.auto.route.BaseRoute;
import org.frogforce503.lib.planning.planned_path.PlannedPath;
import org.frogforce503.robot2025.FieldInfo;
import org.frogforce503.robot2025.commands.drive.DrivePlannedPath;
import org.frogforce503.robot2025.commands.drive.DriveToPose;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.superstructure.SuperstructureMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;

public abstract class AutoMode {
    private final Drive drive;
    private final FieldInfo field;
    private final Superstructure superstructure;

    @Getter private final String name;

    public AutoMode(Drive drive, FieldInfo field, Superstructure superstructure) {
        this.drive = drive;
        this.field = field;
        this.superstructure = superstructure;

        this.name = this.getClass().getSimpleName();
    }

    public abstract Command routine();
    public abstract BaseRoute getRoute();

    public Pose2d getStartingPose(Supplier<Pose2d> override) {
        return getRoute().getInitialPose(override);
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
        return new DrivePlannedPath(drive, field, path);
    }

    public Command setSuperstructureMode(SuperstructureMode mode) {
        return
            Commands.runOnce(() -> superstructure.setCurrentMode(mode))
                .ignoringDisable(true);
    }
}
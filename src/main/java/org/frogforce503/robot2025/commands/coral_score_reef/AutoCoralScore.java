package org.frogforce503.robot2025.commands.coral_score_reef;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.frogforce503.lib.commands.DriveFromPathToPose;
import org.frogforce503.lib.commands.DriveToPose;
import org.frogforce503.lib.io.JoystickInputs;
import org.frogforce503.lib.util.ProximityService;
import org.frogforce503.robot2025.commands.superstructure_selection.SuperstructurePreScore;
import org.frogforce503.robot2025.commands.superstructure_selection.SuperstructureScore;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.offsets.OffsetManager;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure.Mode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * A command group that drives the robot to a target while managing scoring actions.
 * 
 * <p>The command performs the following actions (in parallel):
 * <ul>
 *   <li>Prescores until the robot reaches a specified boundary.</li>
 *   <li>Scores when the robot is inside the boundary.</li>
 *   <li>Drives to the target pose if auto-driving is enabled.</li>
 * </ul>
 */
public class AutoCoralScore extends ParallelCommandGroup {
    public AutoCoralScore(
        Drive drive,
        FieldInfo field,
        Superstructure superstructure,
        JoystickInputs inputs,
        Supplier<Mode> superstructureModeSupplier,
        Supplier<Pose2d> robotPose,
        Supplier<Pose2d> target,
        BooleanSupplier insideBoundary,
        BooleanSupplier autoDrivingEnabled
    ) {
        super(
            Commands.either(
                new SuperstructureScore(superstructure, superstructureModeSupplier),
                new SuperstructurePreScore(superstructure, superstructureModeSupplier),
                insideBoundary)
                    .repeatedly(),
            Commands.either(
                new DriveFromPathToPose(
                    drive,
                    field,
                    robotPose,
                    target,
                    inputs.times(0.75)),
                new DriveToPose(
                    drive,
                    field,
                    robotPose,
                    target,
                    inputs.times(0.75)),
                RobotState::isAutonomous)
                    .onlyIf(autoDrivingEnabled));

        addRequirements(drive, superstructure);
    }

    /** For L2, L3, L4 modes. */
    public AutoCoralScore(
        Drive drive,
        FieldInfo field,
        Superstructure superstructure,
        JoystickInputs inputs,
        OffsetManager offsetManager,
        ProximityService proximityService,
        Supplier<Mode> superstructureModeSupplier,
        Supplier<Pose2d> robotPose,
        Supplier<Branch> branchSupplier,
        BooleanSupplier insideBoundary,
        BooleanSupplier autoDrivingEnabled
    ) {
        this(
            drive,
            field,
            superstructure,
            inputs,
            superstructureModeSupplier,
            robotPose,
            branchSupplier
                .get()
                .getTarget(field, proximityService, offsetManager.getOffsetData()),
            insideBoundary,
            autoDrivingEnabled);
    }

    /** For L1 mode. */
    public AutoCoralScore(
        Drive drive,
        FieldInfo field,
        Superstructure superstructure,
        JoystickInputs inputs,
        ProximityService proximityService,
        Supplier<Mode> superstructureModeSupplier,
        Supplier<Pose2d> robotPose,
        Supplier<ReefSide> reefSideSupplier,
        BooleanSupplier insideBoundary,
        BooleanSupplier autoDrivingEnabled
    ) {
        this(
            drive,
            field,
            superstructure,
            inputs,
            superstructureModeSupplier,
            robotPose,
            reefSideSupplier
                .get()
                .getTarget(field),
            insideBoundary,
            autoDrivingEnabled);
    }
}
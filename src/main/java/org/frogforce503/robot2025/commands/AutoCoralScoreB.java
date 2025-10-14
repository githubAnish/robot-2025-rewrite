package org.frogforce503.robot2025.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.frogforce503.lib.auto.builder.PlannedPathGenerator;
import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.lib.planning.planned_path.PlannedPath;
import org.frogforce503.lib.planning.planned_path.Waypoint;
import org.frogforce503.lib.reefscape.Branch;
import org.frogforce503.lib.reefscape.PrescoreBoundary;
import org.frogforce503.robot2025.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.offsets.OffsetManager;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoCoralScoreB extends Command {
    // Requirements
    private final Drive drive;
    private final FieldInfo field;
    private final Superstructure superstructure;
    private final OffsetManager offsetManager;
    
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<Branch> targetBranchSupplier;
    private final PrescoreBoundary prescoreBoundary;

    private final BooleanSupplier autoDrivingEnabled;

    // Constants
    private final Constraints trajectoryConstraints =
        new Constraints(
            Units.feetToMeters(13),
            Units.feetToMeters(19));

    private final double trajectoryFinalVelocityMeterPerSec = 0.7;

    // State
    private ScoringState currentState;

    private Pose2d robotPose;
    private Pose2d targetPose;

    private PlannedPath trajectory;

    private enum ScoringState {
        PLAN_DRIVE_TO_PRESCORE,
        DRIVE_TO_PRESCORE_AND_PRESCORE,
        PLAN_DRIVE_TO_REEF,
        DRIVE_TO_REEF,
        LEVEL_ALIGN,
        FINISHED
    }

    public AutoCoralScoreB(
        Drive drive,
        FieldInfo field,
        Superstructure superstructure,
        OffsetManager offsetManager,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<Branch> targetBranchSupplier,
        PrescoreBoundary prescoreBoundary,
        BooleanSupplier autoDrivingEnabled
    ) {
        this.drive = drive;
        this.field = field;
        this.superstructure = superstructure;
        this.offsetManager = offsetManager;
        this.robotPoseSupplier = robotPoseSupplier;
        this.targetBranchSupplier = targetBranchSupplier;
        this.prescoreBoundary = prescoreBoundary;
        this.autoDrivingEnabled = autoDrivingEnabled;

        addRequirements(drive, superstructure);
    }

    @Override
    public void initialize() {
        ChassisSpeeds currentVelocity = drive.getCurrentVelocity();

        robotPose = robotPoseSupplier.get();
        targetPose = targetBranchSupplier.get().getTarget(drive, field, offsetManager.getOffsetData()).get();
            
        double initialRobotTranslationalVelocity =
            Math.hypot(
                currentVelocity.vxMetersPerSecond,
                currentVelocity.vyMetersPerSecond);

        trajectory =
            PlannedPathGenerator
                .generate(
                    trajectoryConstraints.maxVelocity,
                    trajectoryConstraints.maxAcceleration,
                    initialRobotTranslationalVelocity,
                    trajectoryFinalVelocityMeterPerSec,
                    Waypoint.fromHolonomicPose(robotPose),
                    Waypoint.fromHolonomicPose(targetPose));

        
    }

    @Override
    public void execute() {
        switch (currentState) {
            case DRIVE_TO_PRESCORE_AND_PRESCORE:
                break;
            case DRIVE_TO_REEF:
                break;
            case FINISHED:
                break;
            case LEVEL_ALIGN:
                break;
            case PLAN_DRIVE_TO_PRESCORE:
                break;
            case PLAN_DRIVE_TO_REEF:
                break;
            default:
                break;
            
        }
    }

    @Override
    public boolean isFinished() {
        return currentState == ScoringState.FINISHED;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
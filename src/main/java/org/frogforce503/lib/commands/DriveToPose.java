package org.frogforce503.lib.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frogforce503.lib.io.JoystickInputs;
import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.robot2025.commands.DriveCommands;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPose extends Command {
    private final Drive drive;
    private final FieldInfo field;

    private final Supplier<Pose2d> robotPose;
    private final Supplier<Pose2d> target;

    private final ProfiledPIDController driveController =
        new ProfiledPIDController(
            4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(1.0, DriveConstants.FAST_TRANSLATION_METERS_PER_SECOND * 0.7));
            
    private final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(DriveConstants.FAST_ROTATION_RADIANS_PER_SECOND, DriveConstants.FAST_ROTATION_RADIANS_PER_SECOND * 0.7));

    private double driveErrorAbs = 0.0;
    private double thetaErrorAbs = 0.0;
    private double ffMinRadius = 0.2;
    private double ffMaxRadius = 0.8;
    private Translation2d lastSetpointTranslation;
    private boolean running = false;

    private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
    private DoubleSupplier omegaFF = () -> 0.0;

    public DriveToPose(Drive drive, FieldInfo field, Supplier<Pose2d> robotPose, Supplier<Pose2d> target) {
        this.robotPose = robotPose;
        this.target = target;
        this.drive = drive;
        this.field = field;

        // Enable continuous input for theta controller
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    public DriveToPose(
        Drive drive,
        FieldInfo field,
        Supplier<Pose2d> robotPose,
        Supplier<Pose2d> target,
        Supplier<Translation2d> linearFF,
        DoubleSupplier omegaFF
    ) {
        this(drive, field, robotPose, target);
        this.linearFF = linearFF;
        this.omegaFF = omegaFF;
    }

    public DriveToPose(
        Drive drive,
        FieldInfo field,
        Supplier<Pose2d> robotPose,
        Supplier<Pose2d> target,
        JoystickInputs inputs
    ) {
        this(
            drive,
            field,
            robotPose,
            target,
            () ->
                DriveCommands
                    .getLinearVelocityFromJoysticks(
                        inputs.xSupplier().getAsDouble(),
                        inputs.ySupplier().getAsDouble())
                    .times(field.onRedAlliance() ? -1.0 : 1.0),
            () -> DriveCommands.getOmegaFromJoysticks(inputs.omegaSupplier().getAsDouble()));
    }

    @Override
    public void initialize() {
        Pose2d currentPose = robotPose.get();
        ChassisSpeeds currentVel = drive.getFieldVelocity();
        Translation2d linearFieldVelocity =
            new Translation2d(currentVel.vxMetersPerSecond, currentVel.vyMetersPerSecond);

        driveController.reset(
            currentPose.getTranslation().getDistance(target.get().getTranslation()),
            Math.min(
                0.0,
                -linearFieldVelocity
                    .rotateBy(
                        target
                            .get()
                            .getTranslation()
                            .minus(currentPose.getTranslation())
                            .getAngle()
                            .unaryMinus())
                    .getX()));

        thetaController
            .reset(
                currentPose.getRotation().getRadians(),
                currentVel.omegaRadiansPerSecond);

        lastSetpointTranslation = currentPose.getTranslation();
    }

    @Override
    public void execute() {
        running = true;

        Pose2d currentPose = robotPose.get();
        Pose2d targetPose = target.get();

        double currentDistance =
            currentPose
                .getTranslation()
                .getDistance(
                    targetPose.getTranslation());

        double ffScaler =
            MathUtil.clamp(
                (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
                0.0,
                1.0);

        driveErrorAbs = currentDistance;

        driveController
            .reset(
                lastSetpointTranslation
                    .getDistance(
                        targetPose.getTranslation()),
                driveController
                    .getSetpoint()
                    .velocity);

        double driveVelocityScalar =
            driveController.getSetpoint().velocity * ffScaler + driveController.calculate(driveErrorAbs, 0.0);

        if (currentDistance < driveController.getPositionTolerance())
            driveVelocityScalar = 0.0;

        lastSetpointTranslation =
            new Pose2d(
                targetPose.getTranslation(),
                currentPose
                    .getTranslation()
                    .minus(
                        targetPose.getTranslation())
                    .getAngle())
            .transformBy(
                GeomUtil
                    .toTransform2d(
                        driveController.getSetpoint().position, 0.0))
            .getTranslation();

        // Calculate theta speed
        double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
                + thetaController.calculate(
                        currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        
        if (thetaErrorAbs < thetaController.getPositionTolerance())
            thetaVelocity = 0.0;

        // Command speeds
        var driveVelocity =
            GeomUtil
                .toPose2d(
                    currentPose
                        .getTranslation()
                        .minus(targetPose.getTranslation())
                        .getAngle())
                .transformBy(
                    GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
                .getTranslation();

        // Scale feedback velocities by input ff
        final double linearS = linearFF.get().getNorm() * 3.0;
        final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;

        driveVelocity =
            driveVelocity
                .interpolate(
                    linearFF
                        .get()
                        .times(DriveConstants.FAST_TRANSLATION_METERS_PER_SECOND),
                    linearS);

        thetaVelocity =
            MathUtil.interpolate(
                thetaVelocity,
                omegaFF.getAsDouble() * DriveConstants.FAST_ROTATION_RADIANS_PER_SECOND,
                thetaS);

        // Apply speeds
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(),
                driveVelocity.getY(),
                thetaVelocity,
                currentPose.getRotation()));

        // Log inputs & outputs
        Logger.recordOutput("DriveToPose/Drive Error", driveErrorAbs);
        Logger.recordOutput("DriveToPose/Theta Error", thetaErrorAbs);
        Logger.recordOutput("DriveToPose/Current Pose", currentPose);
        Logger.recordOutput("DriveToPose/Target Pose", targetPose);
        Logger.recordOutput("DriveToPose/Is Finished", isFinished());
    }

    @Override
    public boolean isFinished() {
        return
            target.get().equals(null) ||
            (driveController.atGoal() && thetaController.atGoal());
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        running = false;
    }
    
    /** Checks if the robot is stopped at the final pose. */
    public boolean atGoal() {
        return running && driveController.atSetpoint() && thetaController.atSetpoint();
    }

    /** Checks if the robot pose is within the allowed drive and theta tolerances. */
    public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
        return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
    }
}
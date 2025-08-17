package org.frogforce503.lib.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frogforce503.lib.io.JoystickInputs;
import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPose extends Command {
    private final PIDFConfig drivePID = new PIDFConfig(4.0, 0.0, 0.0);
    private final PIDFConfig thetaPID = new PIDFConfig(4.0, 0.0, 0.0);
    private final Constraints driveConstraints = new Constraints(3.8, 3.0);
    private final Constraints thetaConstraints = new Constraints(DriveConstants.FAST_ROTATION_RADIANS_PER_SECOND, DriveConstants.FAST_ROTATION_RADIANS_PER_SECOND * 0.7);
    private final double driveTolerance = 0.01;
    private final double thetaTolerance = Units.degreesToRadians(1.0);
    private final double ffMinRadius = 0.01;
    private final double ffMaxRadius = 0.4;

    private final Drive drive;
    private final FieldInfo field;

    private final Supplier<Pose2d> robotPose;
    private final Supplier<Pose2d> target;

    private final ProfiledPIDController driveController =
        new ProfiledPIDController(
            0.0,
            0.0,
            0.0,
            new Constraints(0.0, 0.0));
            
    private final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            0.0,
            0.0,
            0.0,
            new Constraints(0.0, 0.0));

    private Translation2d lastSetpointTranslation;

    private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
    private DoubleSupplier omegaFF = () -> 0.0;

    public DriveToPose(Drive drive, FieldInfo field, Supplier<Pose2d> robotPose, Supplier<Pose2d> target) {
        this.drive = drive;
        this.field = field;
        this.robotPose = robotPose;
        this.target = target;

        driveController.setPID(drivePID.kP(), drivePID.kI(), drivePID.kD());
        driveController.setConstraints(driveConstraints);
        driveController.setTolerance(driveTolerance);
        
        thetaController.setPID(thetaPID.kP(), thetaPID.kI(), thetaPID.kD());
        thetaController.setConstraints(thetaConstraints);
        thetaController.setTolerance(thetaTolerance);

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

        driveController
            .reset(
                lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                driveController.getSetpoint().velocity);

        double driveVelocityScalar =
            driveController.getSetpoint().velocity * ffScaler +
            driveController.calculate(currentDistance, 0.0);

        if (currentDistance < driveController.getPositionTolerance()) {
            driveVelocityScalar = 0.0;
        }

        lastSetpointTranslation =
            new Pose2d(
                targetPose.getTranslation(),
                currentPose
                    .getTranslation()
                    .minus(targetPose.getTranslation())
                    .getAngle())
            .transformBy(
                GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
            .getTranslation();

        // Calculate theta speed
        double thetaVelocity =
            thetaController.getSetpoint().velocity * ffScaler +
            thetaController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        double thetaErrorAbs =
            Math.abs(
                currentPose
                    .getRotation()
                    .minus(targetPose.getRotation())
                    .getRadians());
        
        if (thetaErrorAbs < thetaController.getPositionTolerance()) {
            thetaVelocity = 0.0;
        }

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

        // Log data
        Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
        Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
        Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
        Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
        Logger.recordOutput(
            "DriveToPose/Setpoint",
            new Pose2d[] {
                new Pose2d(
                    lastSetpointTranslation,
                    new Rotation2d(thetaController.getSetpoint().position))});
        Logger.recordOutput("DriveToPose/Goal", new Pose2d[] { targetPose });
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
        // Clear logs
        Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
        Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
    }
}
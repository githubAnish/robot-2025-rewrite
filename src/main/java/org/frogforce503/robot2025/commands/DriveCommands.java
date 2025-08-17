package org.frogforce503.robot2025.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.frogforce503.lib.io.JoystickInputs;
import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.drive.DriveConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class DriveCommands {
    public static final double kDeadband = 0.2;

    private static final PIDFConfig thetaPID = new PIDFConfig(5.0, 0.0, 0.4);
    private static final Constraints thetaConstraints = new Constraints(8.0, 20.0);

    private static final double snapToAngleTolerance = 3.0; // Degrees

    private DriveCommands() {}

    public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), kDeadband);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = Math.pow(linearMagnitude, 2);

        // Return new linear velocity
        return
            GeomUtil
                .toPose2d(linearDirection)
                .transformBy(GeomUtil.toTransform2d(linearMagnitude, 0.0))
                .getTranslation();
    }

    public static double getOmegaFromJoysticks(double driverOmega) {
        double omega = MathUtil.applyDeadband(driverOmega, kDeadband);
        return omega * omega * Math.signum(omega);
    }

    /**
     * Field-relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(Drive drive, FieldInfo field, JoystickInputs inputs, BooleanSupplier robotRelative) {
        return Commands.run(() -> {
            // Get linear velocity
            Translation2d linearVelocity =
                getLinearVelocityFromJoysticks(inputs.xSupplier().getAsDouble(), inputs.ySupplier().getAsDouble());

            // Calculate angular velocity
            double omega =
                getOmegaFromJoysticks(inputs.omegaSupplier().getAsDouble());

            BooleanSupplier slowModeEnabled = drive::isSlowModeEnabled;

            // Calculate max linear velocity
            double maxLinearVelocity =
                slowModeEnabled.getAsBoolean()
                    ? DriveConstants.SLOW_TRANSLATION_METERS_PER_SECOND
                    : DriveConstants.FAST_TRANSLATION_METERS_PER_SECOND;

            // Calculate max rotational velocity
            double maxOmega =
                slowModeEnabled.getAsBoolean()
                    ? DriveConstants.SLOW_ROTATION_RADIANS_PER_SECOND
                    : DriveConstants.FAST_ROTATION_RADIANS_PER_SECOND;

            ChassisSpeeds speeds =
                new ChassisSpeeds(
                    linearVelocity.getX() * maxLinearVelocity,
                    linearVelocity.getY() * maxLinearVelocity,
                    omega * maxOmega);

            drive.runVelocity(
                robotRelative.getAsBoolean()
                    ? speeds
                    : ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        field.onRedAlliance()
                            ? drive.getAngle().plus(Rotation2d.kPi)
                            : drive.getAngle()));
        }, drive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(Drive drive, FieldInfo field, JoystickInputs inputs, Supplier<Rotation2d> rotationSupplier) {
        // Create PID controller
        ProfiledPIDController angleController =
            new ProfiledPIDController(
                thetaPID.kP(),
                thetaPID.kI(),
                thetaPID.kD(),
                thetaConstraints);

        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                () -> {
                // Get linear velocity
                Translation2d linearVelocity =
                    getLinearVelocityFromJoysticks(inputs.xSupplier().getAsDouble(), inputs.ySupplier().getAsDouble());

                // Calculate angular speed
                Rotation2d rotation = drive.getAngle();
                double omega =
                    angleController.calculate(
                        rotation.getRadians(), rotationSupplier.get().getRadians());

                // Convert to field relative speeds & send command
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        linearVelocity.getX() * DriveConstants.FAST_TRANSLATION_METERS_PER_SECOND,
                        linearVelocity.getY() * DriveConstants.FAST_TRANSLATION_METERS_PER_SECOND,
                        omega);

                boolean isFlipped = field.onRedAlliance();

                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds, isFlipped ? rotation.plus(Rotation2d.kPi) : rotation));
                }, drive)

        // Reset PID controller when command starts
        .beforeStarting(
            () -> angleController.reset(drive.getAngle().getRadians()));
    }

    public static Command snapToAngle(Drive drive, FieldInfo field, Supplier<Rotation2d> rotationSupplier) {
        return
            joystickDriveAtAngle(drive, field, JoystickInputs.kZero, rotationSupplier)
            .until(
                () ->
                    Math.abs(drive.getAngle().minus(rotationSupplier.get()).getDegrees()) < snapToAngleTolerance);
    }
}
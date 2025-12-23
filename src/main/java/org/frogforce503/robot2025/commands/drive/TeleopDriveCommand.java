package org.frogforce503.robot2025.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.frogforce503.lib.io.JoystickInputs;
import org.frogforce503.robot2025.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.drive.DriveConstants;

public class TeleopDriveCommand extends Command {
    // Requirements
    private final Drive drive;
    private final JoystickInputs inputs;
    private final BooleanSupplier robotRelative;
    private final BooleanSupplier slowMode;
    private final DoubleSupplier teleopTranslationScalar;
    private final DoubleSupplier teleopRotationScalar;

    public TeleopDriveCommand(Drive drive, JoystickInputs inputs) {
        this.drive = drive;
        this.inputs = inputs;
        this.robotRelative = drive::isRobotRelative;
        this.slowMode = drive::isSlowMode;
        this.teleopTranslationScalar = drive::getTeleopTranslationScalar;
        this.teleopRotationScalar = drive::getTeleopRotationScalar;

        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Get driver input velocities
        Translation2d driverLinearVelocity = inputs.getLinearVelocityFromJoysticks();
        double driverOmega = inputs.getOmegaFromJoysticks();

        // Determine max velocities based on slow mode
        final double maxLinearVelocity =
            slowMode.getAsBoolean() // use the teleop translation scalar here
                ? DriveConstants.slowModeLinearSpeed
                : DriveConstants.maxLinearSpeed;

        final double maxOmega =
            slowMode.getAsBoolean() // use the teleop rotation scalar here
                ? DriveConstants.slowModeOmega
                : DriveConstants.maxOmega;

        // Calculate speeds
        double xVelocity = driverLinearVelocity.getX() * maxLinearVelocity * teleopTranslationScalar.getAsDouble();
        double yVelocity = driverLinearVelocity.getY() * maxLinearVelocity * teleopTranslationScalar.getAsDouble();
        double omega = driverOmega * maxOmega * teleopRotationScalar.getAsDouble();

        // Apply speeds to drivetrain
        ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, omega);

        drive.runVelocity(
            robotRelative.getAsBoolean()
                ? speeds
                : ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds,
                    FieldInfo.isRed()
                        ? drive.getAngle().plus(Rotation2d.kPi)
                        : drive.getAngle()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
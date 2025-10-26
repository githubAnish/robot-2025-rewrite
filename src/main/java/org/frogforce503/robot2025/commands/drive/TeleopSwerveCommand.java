package org.frogforce503.robot2025.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

import org.frogforce503.lib.io.JoystickInputs;
import org.frogforce503.robot2025.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.drive.DriveConstants;

public class TeleopSwerveCommand extends Command {
    private final double HEADING_HOLD_DELAY = 0.25;

    private final Drive drive;
    private final FieldInfo field;
    private final JoystickInputs inputs;
    private final BooleanSupplier robotRelative;
    private final BooleanSupplier slowMode;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(6.0);

    private final PIDController headingController = new PIDController(4.0, 0.0, 0.45);
    private Rotation2d targetHeading = new Rotation2d();
    private double lastManualRotTime = 0.0;

    public TeleopSwerveCommand(
        Drive drive,
        FieldInfo field,
        JoystickInputs inputs,
        BooleanSupplier robotRelative,
        BooleanSupplier slowMode
    ) {
        this.drive = drive;
        this.field = field;
        this.inputs = inputs;
        this.robotRelative = robotRelative;
        this.slowMode = slowMode;

        headingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        targetHeading = drive.getAngle();
        lastManualRotTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        // Get driver joystick inputs
        Translation2d driverLinearVelocity = inputs.getLinearVelocityFromJoysticks();
        double driverOmega = inputs.getOmegaFromJoysticks();

        // Compute speed limits
        double maxLinearVelocity =
            slowMode.getAsBoolean()
                ? DriveConstants.SLOW_TRANSLATION_METERS_PER_SECOND
                : DriveConstants.FAST_TRANSLATION_METERS_PER_SECOND;

        double maxOmega =
            slowMode.getAsBoolean()
                ? DriveConstants.SLOW_ROTATION_RADIANS_PER_SECOND
                : DriveConstants.FAST_ROTATION_RADIANS_PER_SECOND;

        // Apply slew rate limiting
        double xVelocity = xLimiter.calculate(driverLinearVelocity.getX()) * maxLinearVelocity;
        double yVelocity = yLimiter.calculate(driverLinearVelocity.getY()) * maxLinearVelocity;
        double omega = 0.0;

        // Heading hold logic with short delay
        if (Math.abs(driverOmega) > 0.05) {
            omega = driverOmega * maxOmega;
            targetHeading = drive.getAngle();
            lastManualRotTime = Timer.getFPGATimestamp();

        } else if (Timer.getFPGATimestamp() - lastManualRotTime > HEADING_HOLD_DELAY) {
            omega =
                headingController.calculate(
                    drive.getAngle().getRadians(),
                    targetHeading.getRadians());

        } else {
            omega = 0.0; // Free drift before PID re-engages
        }

        // Smooth omega transitions
        omega = omegaLimiter.calculate(omega);

        ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, omega);

        drive.runVelocity(
            robotRelative.getAsBoolean()
                ? speeds
                : ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds,
                    field.onRedAlliance()
                        ? drive.getAngle().plus(Rotation2d.kPi)
                        : drive.getAngle()));
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
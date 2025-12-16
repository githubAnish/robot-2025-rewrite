package org.frogforce503.robot2025.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

import org.frogforce503.lib.io.JoystickInputs;
import org.frogforce503.robot2025.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.drive.DriveConstants;

public class TeleopSwerveCommand extends Command {
    // Constants
    private final double DEADBAND = 0.03;
    private final double HEADING_HOLD_DELAY = 0.02113;
    private final double ANGULAR_VELOCITY_THRESHOLD = Units.degreesToRadians(10);

    // Requirements
    private final Drive drive;
    private final JoystickInputs inputs;
    private final BooleanSupplier robotRelative;
    private final BooleanSupplier slowMode;

    // Rate limiters
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(18.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(18.0);
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(36.0);

    // Heading control
    private final PIDController headingController = new PIDController(4.0, 0.0, 0.15);
    private Rotation2d targetHeading = new Rotation2d();
    private double lastManualRotTime = 0.0;

    public TeleopSwerveCommand(Drive drive, JoystickInputs inputs) {
        this.drive = drive;
        this.inputs = inputs;
        this.robotRelative = drive::isRobotRelative;
        this.slowMode = drive::isSlowMode;

        headingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        targetHeading = drive.getAngle();
        lastManualRotTime = Timer.getFPGATimestamp();

        headingController.reset();
    }

    @Override
    public void execute() {
        // Get driver input velocities
        Translation2d driverLinearVelocity = inputs.getLinearVelocityFromJoysticks();
        double driverOmega = inputs.getOmegaFromJoysticks();

        // Determine max velocities based on slow mode
        final double maxLinearVelocity =
            slowMode.getAsBoolean()
                ? DriveConstants.slowModeLinearSpeed
                : DriveConstants.maxLinearSpeed;

        final double maxOmega =
            slowMode.getAsBoolean()
                ? DriveConstants.slowModeOmega
                : DriveConstants.maxOmega;

        // Apply rate limiting
        double xVelocity = xLimiter.calculate(driverLinearVelocity.getX() * maxLinearVelocity);
        double yVelocity = yLimiter.calculate(driverLinearVelocity.getY() * maxLinearVelocity);
        double omega = 0.0;

        // Determine angular velocity
        Rotation2d currentHeading = drive.getAngle();

        // Manual heading stabilization
        boolean driverRotating = Math.abs(driverOmega) > DEADBAND;
        boolean withinHeadingHoldDelay = Timer.getFPGATimestamp() - lastManualRotTime < HEADING_HOLD_DELAY;
        boolean robotRotating = Math.abs(drive.getRobotVelocity().omegaRadiansPerSecond) > ANGULAR_VELOCITY_THRESHOLD;

        if (driverRotating || (withinHeadingHoldDelay && robotRotating)) {
            // Driver is rotating
            omega = omegaLimiter.calculate(driverOmega * maxOmega);
            targetHeading = currentHeading;
            lastManualRotTime = Timer.getFPGATimestamp();

        } else {
            // Stabilize to last heading
            omega =
                headingController.calculate(
                    currentHeading.getRadians(),
                    targetHeading.getRadians());
        }

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
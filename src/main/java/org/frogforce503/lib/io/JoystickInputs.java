package org.frogforce503.lib.io;

import java.util.function.DoubleSupplier;

import org.frogforce503.lib.math.GeomUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public record JoystickInputs(
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    DoubleSupplier omegaSupplier
) {
    public static final double kDeadband = 0.2;
    public static final JoystickInputs kZero = new JoystickInputs(() -> 0, () -> 0, () -> 0);

    public JoystickInputs(CommandXboxController joystick) {
        this(
            () -> -joystick.getLeftY(),
            () -> -joystick.getLeftX(),
            () -> -joystick.getRightX());
    }

    public JoystickInputs times(double scalar) {
        return
            new JoystickInputs(
                () -> xSupplier.getAsDouble() * scalar,
                () -> ySupplier.getAsDouble() * scalar,
                () -> omegaSupplier.getAsDouble() * scalar);
    }

    public Translation2d getLinearVelocityFromJoysticks() {
        double x = xSupplier.getAsDouble();
        double y = ySupplier.getAsDouble();

        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), kDeadband);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return
            GeomUtil
                .toPose2d(linearDirection)
                .transformBy(GeomUtil.toTransform2d(linearMagnitude, 0.0))
                .getTranslation();
    }

    public double getOmegaFromJoysticks() {
        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), kDeadband);
        return omega * omega * Math.signum(omega);
    }
}
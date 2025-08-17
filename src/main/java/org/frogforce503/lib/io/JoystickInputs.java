package org.frogforce503.lib.io;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public record JoystickInputs(
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    DoubleSupplier omegaSupplier
) {
    public static JoystickInputs kZero =
        new JoystickInputs(
            () -> 0,
            () -> 0,
            () -> 0);

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
}
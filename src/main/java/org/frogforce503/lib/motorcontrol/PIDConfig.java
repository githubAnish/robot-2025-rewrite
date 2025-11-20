package org.frogforce503.lib.motorcontrol;

import org.frogforce503.lib.math.Range;

import edu.wpi.first.math.controller.PIDController;

public record PIDConfig(
    double kP,
    double kI,
    double kD,
    double kIZone,
    Range kIntegratorRange
) {
    public PIDConfig(double kP, double kI, double kD, double kIZone) {
        this(kP, kI, kD, kIZone, new Range(-1.0, 1.0));
    }

    public PIDConfig(double kP, double kI, double kD) {
        this(kP, kI, kD, Double.POSITIVE_INFINITY, new Range(-1.0, 1.0));
    }

    public PIDController toPIDController() {
        PIDController controller = new PIDController(kP, kI, kD);
        controller.setIZone(kIZone);
        controller.setIntegratorRange(kIntegratorRange.min(), kIntegratorRange.max());
        return controller;
    }
}
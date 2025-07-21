package org.frogforce503.lib.control.speed;

public record SpeedConstraintsConfig(
        double maxVelocityMetersPerSec,
        double maxAccelerationMetersPerSec2) {}
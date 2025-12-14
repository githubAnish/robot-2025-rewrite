package org.frogforce503.robot2025.subsystems.superstructure.claw;

import edu.wpi.first.math.util.Units;

public final class ClawConstants {
    public static final double kTolerance = Units.rotationsPerMinuteToRadiansPerSecond(25.0);

    public static final double START = 0;
    public static final double INTAKE_CORAL = 0;

    public static final double ALGAE_PLUCK_LOW = Units.inchesToMeters(START);
    public static final double ALGAE_PLUCK_HIGH = Units.inchesToMeters(START);
}
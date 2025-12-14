package org.frogforce503.robot2025.subsystems.superstructure.elevator;

import org.frogforce503.robot2025.Robot;

import edu.wpi.first.math.util.Units;

public final class ElevatorConstants {
    public static final double kTolerance = Units.inchesToMeters(0.5);

    public static final double minHeight = Robot.bot.getElevatorConfig().minHeight();
    public static final double maxHeight = Robot.bot.getElevatorConfig().maxHeight();

    public static final double START = minHeight;

    public static final double ALGAE_PLUCK_LOW = Units.inchesToMeters(START);
    public static final double ALGAE_PLUCK_HIGH = Units.inchesToMeters(START);
}
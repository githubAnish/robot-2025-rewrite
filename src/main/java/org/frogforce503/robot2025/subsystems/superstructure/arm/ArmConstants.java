package org.frogforce503.robot2025.subsystems.superstructure.arm;

import org.frogforce503.robot2025.Robot;

import edu.wpi.first.math.util.Units;

public final class ArmConstants {
    public static final double kTolerance = Units.degreesToRadians(0.5);

    public static final double minAngle = Robot.bot.getArmConfig().minAngle();
    public static final double maxAngle = Robot.bot.getArmConfig().maxAngle();

    public static final double START = Units.degreesToRadians(-89);
    public static final double STOW = START;

    public static final double ALGAE_PLUCK_LOW = START;
    public static final double ALGAE_PLUCK_HIGH = START;
}
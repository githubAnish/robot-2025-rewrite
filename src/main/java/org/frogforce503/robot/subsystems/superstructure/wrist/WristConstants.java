package org.frogforce503.robot.subsystems.superstructure.wrist;

import org.frogforce503.robot.Robot;

import edu.wpi.first.math.util.Units;

public final class WristConstants {
    public static final double kTolerance = Units.degreesToRadians(3.0);

    public static final double minAngle = Robot.bot.getWristConfig().minAngle();
    public static final double maxAngle = Robot.bot.getWristConfig().maxAngle();

    public static final double START = Units.degreesToRadians(0);
    public static final double INTAKE_CORAL = Units.degreesToRadians(-36.0);

    public static final double ALGAE_PLUCK_LOW = START;
    public static final double ALGAE_PLUCK_HIGH = START;
}
package org.frogforce503.robot2025.subsystems.superstructure.intakepivot;

import org.frogforce503.robot2025.Robot;

import edu.wpi.first.math.util.Units;

public class IntakePivotConstants {
    public static final double kPivotTolerance = Units.degreesToRadians(0.5);

    public static final double START = Units.degreesToRadians(90);

    public static final double minAngle = Robot.bot.getIntakePivotConfig().minAngle();
    public static final double maxAngle = Robot.bot.getIntakePivotConfig().maxAngle();

    public static final double INTAKE_CLEARANCE = Units.degreesToRadians(84);
    public static final double ARM_STOW_CLEARANCE = Units.degreesToRadians(37);
}
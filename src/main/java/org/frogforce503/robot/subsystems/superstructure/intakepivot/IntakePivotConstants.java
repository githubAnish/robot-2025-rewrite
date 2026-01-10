package org.frogforce503.robot.subsystems.superstructure.intakepivot;

import org.frogforce503.robot.Robot;

import edu.wpi.first.math.util.Units;

public class IntakePivotConstants {
    public static final double kPivotTolerance = Units.degreesToRadians(3.0);

    public static final double minAngle = Robot.bot.getIntakePivotConfig().minAngle();
    public static final double maxAngle = Robot.bot.getIntakePivotConfig().maxAngle();

    public static final double START = maxAngle;

    public static final double INTAKE_CLEARANCE = Units.degreesToRadians(84);
    public static final double ARM_STOW_CLEARANCE = Units.degreesToRadians(37);
}
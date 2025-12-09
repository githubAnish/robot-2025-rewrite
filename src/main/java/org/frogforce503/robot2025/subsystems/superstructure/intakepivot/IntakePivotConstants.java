package org.frogforce503.robot2025.subsystems.superstructure.intakepivot;

import org.frogforce503.robot2025.Robot;

import edu.wpi.first.math.util.Units;

public class IntakePivotConstants {
    public static final double kPivotTolerance = 1.0;

    public static final double START = Units.degreesToRadians(90);

    public static final double minAngle = Robot.bot.getIntakePivotConfig().minAngle();
    public static final double maxAngle = Robot.bot.getIntakePivotConfig().maxAngle();

    public static final double NONE = 0.0;

    public static final double INTAKE_CLEARANCE = 187.0;
    public static final double SCORE_CLEARANCE = 137.0;

    public static final double INTAKE_ALGAE_FROM_GROUND = 115.0;
    public static final double HOLD_ALGAE = 187.0;

    public static final double IDLE = 193.0;

    public static final double HANDOFF_RELEASE = 187.0;
    public static final double HANDOFF_EJECT = 187.0;

    public static final double PROCESSOR_FROM_INTAKE = 180.0;
    public static final double PROCESSOR_EJECT_ALGAE = 180.0;

    public static final double LOW_CLEARANCE = 187.0;
    public static final double LOW_CLEARANCE_AUTON = 137.0;
    public static final double SCORING_CLEARANCE = 137.0;

    public static final double CORAL_HOLD = 170.0;

    public static final double INTAKE = 115.0;
    public static final double HOLD = 187.0;
    public static final double HOLD_CLAW = 187.0;

    public static final double HANDOFF = 187.0;
}
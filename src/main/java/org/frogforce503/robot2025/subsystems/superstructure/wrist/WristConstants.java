package org.frogforce503.robot2025.subsystems.superstructure.wrist;

import org.frogforce503.robot2025.Robot;

import edu.wpi.first.math.util.Units;

public final class WristConstants {
    public static final double kTolerance = 1.0;
    
    public static final double START = Units.degreesToRadians(0.0); // straight horizontally
    public static final double PARALLEL_TO_GROUND = 90.0;

    public static final double minAngle = Robot.bot.getWristConfig().minAngle();
    public static final double maxAngle = Robot.bot.getWristConfig().maxAngle();

    public static final double TOZERO = 24;
    public static final double TO90 = 90;

    public static final double PRESCORE_L1 = 0;
    public static final double PRESCORE_L2 = 0;
    public static final double PRESCORE_L3 = 0;
    public static final double PRESCORE_L4 = 0;

    public static final double SCORE_L1 = 0;
    public static final double SCORE_L2 = 0;
    public static final double SCORE_L3 = 0;
    public static final double SCORE_L4 = 0;

    public static final double PLUCK_ALGAE_HIGH = 0;
    public static final double PLUCK_ALGAE_LOW = 0;

    public static final double INTAKE_CORAL = 54;

    public static final double HOLD_ALGAE = 30;

    public static final double HANDOFF = 10;

    public static final double HANDOFF_RELEASE = 10;

    public static final double PROCESSOR_FROM_CLAW = 45;

    public static final double POSTSCORE_L4 = 0;

    public static final double BARGE = 0;

    public static final double NET_RELEASE = 230;
}
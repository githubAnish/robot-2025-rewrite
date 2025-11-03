package org.frogforce503.robot2025.subsystems.superstructure.arm;

import lombok.Getter;

public enum ArmGoal {
    START(0.0),

    NONE(0),

    DOWN(18),

    PRESCORE_L1(0),
    PRESCORE_L2(0),
    PRESCORE_L3(0),
    PRESCORE_L4(30),

    SCORE_L1(0),
    SCORE_L2(0),
    SCORE_L3(0),
    SCORE_L4(1),

    POSTSCORE_L4(0),

    PLUCK_ALGAE_HIGH(0),
    PLUCK_ALGAE_LOW(0),

    HOLD_ALGAE(70),

    PROCESSOR_FROM_CLAW(70),

    NET_RELEASE(180),

    HANDOFF(130), // handoff is like arm 130 deg
    HANDOFF_RELEASE(70),

    BARGE(0);
    
    @Getter private double angle;

    private ArmGoal(double angle) { // Converts degrees to Rotation2d
        this.angle = angle;
    }
}
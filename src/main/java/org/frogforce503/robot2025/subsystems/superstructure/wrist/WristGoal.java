package org.frogforce503.robot2025.subsystems.superstructure.wrist;

import lombok.Getter;

public enum WristGoal {
    START(0.0),
    PARALLEL_TO_GROUND(90.0),

    TOZERO(24),
    TO90(90),

    PRESCORE_L1(0),
    PRESCORE_L2(0),
    PRESCORE_L3(0),
    PRESCORE_L4(0),

    SCORE_L1(0),
    SCORE_L2(0),
    SCORE_L3(0),
    SCORE_L4(0),

    PLUCK_ALGAE_HIGH(0),
    PLUCK_ALGAE_LOW(0),

    INTAKE_CORAL(54),

    HOLD_ALGAE(30),

    HANDOFF(10),

    HANDOFF_RELEASE(10),

    PROCESSOR_FROM_CLAW(45),

    POSTSCORE_L4(0),

    BARGE(0),

    NET_RELEASE(230);
    
    @Getter private double angle;

    private WristGoal(double position) {
        this.angle = position;
    }
}
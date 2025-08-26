package org.frogforce503.robot2025.subsystems.superstructure.elevator;

public enum ElevatorGoal {
    DOWN(0),

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

    HOLD_CORAL(10),
    HOLD_ALGAE(0),

    HANDOFF(17),
    HANDOFF_RELEASE(0),

    BARGE(0),

    SWITCH_MODE(20);
    
    public double position;

    private ElevatorGoal(double position) {
        this.position = position;
    }
}
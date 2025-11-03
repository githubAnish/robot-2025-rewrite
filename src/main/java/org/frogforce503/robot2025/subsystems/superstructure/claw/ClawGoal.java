package org.frogforce503.robot2025.subsystems.superstructure.claw;

import lombok.Getter;

public enum ClawGoal {
    START(0.0),

    NONE(0),

    OFF(0),

    INTAKE_CORAL(0),
    INTAKE_ALGAE(0),

    HOLD_ALGAE(0),
    
    EJECT_CORAL(0),
    EJECT_CORAL_FOR_L1(0, 0),

    EJECT_ALGAE(0),
    SLOW_EJECT_ALGAE(0),

    HANDOFF_INTAKE_TO_CLAW(0);
    
    @Getter private double leftVelocity, rightVelocity;

    private ClawGoal(double leftVelocity, double rightVelocity) {
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
    }

    private ClawGoal(double velocity) {
        this(velocity, velocity);
    }
}
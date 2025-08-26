package org.frogforce503.robot2025.subsystems.superstructure.claw;

public enum ClawGoal {
    OFF(0),

    INTAKE_CORAL(0),
    INTAKE_ALGAE(0),

    HOLD_ALGAE(0),
    
    EJECT_CORAL(0),
    EJECT_CORAL_FOR_L1(0, 0),

    EJECT_ALGAE(0),
    SLOW_EJECT_ALGAE(0),

    HANDOFF_INTAKE_TO_CLAW(0);
    
    public double velocityLeft, velocityRight;

    private ClawGoal(double velocityLeft, double velocityRight) {
        this.velocityLeft = velocityLeft;
        this.velocityRight = velocityRight;
    }

    private ClawGoal(double volts) {
        this(volts, volts);
    }
}
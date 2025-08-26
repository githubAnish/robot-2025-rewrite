package org.frogforce503.robot2025.subsystems.superstructure.intake;

public enum IntakeGoal {
    INTAKE_CLEARANCE(187),
    SCORE_CLEARANCE(137),

    INTAKE_ALGAE_FROM_GROUND(115, 3000.0*12.0/10000.0),
    HOLD_ALGAE(187, 1000.0*12.0/10000.0),

    IDLE(193),

    HANDOFF_RELEASE(187, -500*12/10000),
    HANDOFF_EJECT(187, -500*12/10000),

    PROCESSOR_FROM_INTAKE(180),
    PROCESSOR_EJECT_ALGAE(180, -500*12/10000), // Eject algae from intake rollers

    LOW_CLEARANCE(187),
    LOW_CLEARANCE_AUTON(137),
    SCORING_CLEARANCE(137),

    CORAL_HOLD(170),

    INTAKE(115, 3000*12/10000),
    HOLD(187, 1000*12/10000),
    HOLD_CLAW(187),

    HANDOFF(187, 500*12/10000),

    UP_PIVOT(-0.2), // pct, volts
    DOWN_PIVOT(0.7); // pct volts
    
    public double pivotPosition, rollerVolts;
    
    private IntakeGoal(double pivotPosition, double rollerVolts) {
        this.pivotPosition = pivotPosition;
        this.rollerVolts = rollerVolts;
    }

    private IntakeGoal(double pivotPosition) {
        this.pivotPosition = pivotPosition;
        this.rollerVolts = 0.0;
    }
}
package org.frogforce503.robot2025.subsystems.superstructure.intake;

public class IntakeConstants {
    public static final double kPivotTolerance = 1.0;
    public static final double kRollerTolerance = 0.5;

    public static final IntakeGoal START = new IntakeGoal(0.0, 0.0);

    public static final IntakeGoal NONE = new IntakeGoal(0.0, 0.0);

    public static final IntakeGoal INTAKE_CLEARANCE = new IntakeGoal(187.0, 0.0);
    public static final IntakeGoal SCORE_CLEARANCE = new IntakeGoal(137.0, 0.0);

    public static final IntakeGoal INTAKE_ALGAE_FROM_GROUND = new IntakeGoal(115.0, 3000.0*12.0/10000.0);
    public static final IntakeGoal HOLD_ALGAE = new IntakeGoal(187.0, 1000.0*12.0/10000.0);

    public static final IntakeGoal IDLE = new IntakeGoal(193.0, 0.0);

    public static final IntakeGoal HANDOFF_RELEASE = new IntakeGoal(187.0, -500.0*12/10000);
    public static final IntakeGoal HANDOFF_EJECT = new IntakeGoal(187.0, -500.0*12/10000);

    public static final IntakeGoal PROCESSOR_FROM_INTAKE = new IntakeGoal(180.0, 0.0);
    public static final IntakeGoal PROCESSOR_EJECT_ALGAE = new IntakeGoal(180.0, -500.0*12/10000); // Eject algae from intake rollers

    public static final IntakeGoal LOW_CLEARANCE = new IntakeGoal(187.0, 0.0);
    public static final IntakeGoal LOW_CLEARANCE_AUTON = new IntakeGoal(137.0, 0.0);
    public static final IntakeGoal SCORING_CLEARANCE = new IntakeGoal(137.0, 0.0);

    public static final IntakeGoal CORAL_HOLD = new IntakeGoal(170.0, 0.0);

    public static final IntakeGoal INTAKE = new IntakeGoal(115.0, 3000.0*12/10000);
    public static final IntakeGoal HOLD = new IntakeGoal(187.0, 1000.0*12/10000);
    public static final IntakeGoal HOLD_CLAW = new IntakeGoal(187.0, 0.0);

    public static final IntakeGoal HANDOFF = new IntakeGoal(187.0, 500.0*12/10000);

    public record IntakeGoal(double pivotAngle, double rollerVelocity) {}
}

package org.frogforce503.robot2025.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Define the high-level methods needed in {@code Superstructure.java} here.
 * This class is purely for organizational purposes and helps serve as a method planner for the superstructure.
 */
public interface SuperstructureBaseFactory {
    // Coral
    /** Move intake out so arm can come down, then get elevator, arm, wrist down, then move intake back in, then run claw, then wait until lower beam break hit */
    Command intakeCoral();

    Command preScoreL1(); // Be in L1 position
    Command preScoreL2(); // Be in L1 position
    Command preScoreL3(); // Be in L1 position
    Command preScoreL4(); // Raise arm up to L4 height & wrist pointed up

    Command scoreL1(); // Raise elevator, arm, & wrist to L1 setpoints
    Command scoreL2(); // Raise elevator, arm, & wrist to L2 setpoints
    Command scoreL3(); // Raise elevator, arm, & wrist to L3 setpoints
    Command scoreL4(); // Raise elevator, arm, & wrist to L4 setpoints

    Command ejectCoral();
    Command ejectCoralForL1();

    // Algae
    Command intakeAlgaeFromGround(); // StartEndCommand - Start: Pivot Down & Rollers On - End: Pivot Up and Rollers Off
    Command holdAlgaeFromGround();

    Command intakeAlgaeFromHandoff(); // Bring intake down, put elevator at 0, arm at 90 deg from ground, wrist pointed down
    Command holdAlgaeFromHandoff(); // Go to algae holding position (bring elevator down)

    Command pluckHighAlgae(); // Raise elevator, arm, & wrist to High Pluck setpoints
    Command pluckLowAlgae(); // Raise elevator, arm, & wrist to Low Pluck setpoints
    Command holdAlgaeFromPluck();

    Command scoreProcessor(); // Decide based on if algae in claw
    Command scoreProcessorFromIntake(); // Eject algae
    Command scoreProcessorFromClaw(); // Point arm & wrist towards processor & eject algae

    Command scoreBarge();

    Command ejectAlgaeFromIntake();
    Command ejectAlgaeFromClaw();

    // Homing
    Command home(); // Bring elevator down, 
    Command homeAfterL4(); // Bring elevator down, but keep arm up & wrist pointed up
}
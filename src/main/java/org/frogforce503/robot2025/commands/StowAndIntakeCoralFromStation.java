package org.frogforce503.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class StowAndIntakeCoralFromStation extends Command {
    // // Requirements
    // private final Drive drive;
    // private final FieldInfo field;

    // private final Superstructure superstructure;
    // private final Elevator elevator;
    // private final Arm arm;
    // private final Wrist wrist;
    // private final Claw claw;
    // private final Intake intake;

    // private final Leds leds;

    // private final PrescoreBoundary prescoreBoundary;

    // // State
    // private DrivePlannedPath driveToStation;

    // private IntakingState currentState = IntakingState.SAFE_DISTANCE_FROM_REEF;

    // private enum IntakingState {
    //     SAFE_DISTANCE_FROM_REEF,
    //     PUT_INTAKEPIVOT_OUT,
    //     PARTIAL_STOW,
    //     STOW,
    //     WAIT_FOR_LOWER_TRUE,
    //     WAIT_FOR_LOWER_FALSE,
    //     FINISHED
    // }

    // public StowAndIntakeCoralFromStation(Drive drive, FieldInfo field, Superstructure superstructure, Leds leds) {
    //     this.drive = drive;
    //     this.field = field;

    //     this.superstructure = superstructure;
    //     this.elevator = superstructure.getElevator();
    //     this.arm = superstructure.getArm();
    //     this.wrist = superstructure.getWrist();
    //     this.claw = superstructure.getClaw();
    //     this.intake = superstructure.getIntake();

    //     this.leds = leds;

    //     this.prescoreBoundary = new PrescoreBoundary(drive, field);

    //     addRequirements(drive, elevator, arm, wrist, claw, intake, leds);
    // }

    // @Override
    // public void initialize() {
    //     if (superstructure.isHasCoral()) {
    //         currentState = IntakingState.FINISHED;
    //     }

    //     Station closestStation = ProximityUtil.getClosestStation(drive, field);
        
    //     PlannedPath pathToStation =
    //         PlannedPathGenerator.generate(
    //             0,
    //             0,
    //             0,
    //             0,
    //             Waypoint.fromHolonomicPose(drive.getCurrentPose()),
    //             Waypoint.fromHolonomicPose(closestStation.getTarget(field).get()));
        
    //     driveToStation = new DrivePlannedPath(drive, field, pathToStation);
    //     driveToStation.initialize();
    // }

    // @Override
    // public void execute() {
    //     // switch (currentState) {
    //     //     case SAFE_DISTANCE_FROM_REEF:
    //     //         if (!prescoreBoundary.insideBoundary()) {
    //     //             currentState = IntakingState.PUT_INTAKEPIVOT_OUT;
    //     //         }
    //     //         break;

    //     //     case PUT_INTAKEPIVOT_OUT:
    //     //         intake.setGoal(IntakeGoal.SCORE_CLEARANCE);
    //     //         if (intake.isPivotAtAngle(IntakeGoal.SCORE_CLEARANCE.getPivotAngle())) {
    //     //             currentState = IntakingState.PARTIAL_STOW;
    //     //         }
    //     //         break;

    //     //     case PARTIAL_STOW:
    //     //         elevator.setGoal(ElevatorGoal.DOWN);
    //     //         currentState = IntakingState.STOW;
    //     //         break;

    //     //     case STOW:
    //     //         arm.setAngle(ArmConstants.DOWN);
    //     //         wrist.setGoal(WristGoal.INTAKE_CORAL);
    //     //         claw.setGoal(ClawGoal.INTAKE_CORAL);
    //     //         currentState = IntakingState.WAIT_FOR_LOWER_TRUE;
    //     //         break;
                
    //     //     case WAIT_FOR_LOWER_TRUE:
    //     //         intake.setGoal(IntakeGoal.INTAKE_CLEARANCE);
    //     //         if (superstructure.lowerSensorTripped()) {
    //     //             currentState = IntakingState.WAIT_FOR_LOWER_FALSE;
    //     //         }
    //     //         break;

    //     //     case WAIT_FOR_LOWER_FALSE:
    //     //         if (!superstructure.lowerSensorTripped() && claw.coralCurrentThresholdForIntookMet()) {
    //     //             currentState = IntakingState.FINISHED;
    //     //         }
    //     //         break;

    //     //     case FINISHED:
    //     //         claw.stop();
    //     //         superstructure.setHasCoral(true);
    //     //         break;    
    //     // }

    //     driveToStation.execute();
    // }

    // @Override
    // public boolean isFinished() {
    //     return currentState == IntakingState.FINISHED;
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     driveToStation.end(interrupted);
    // }
}
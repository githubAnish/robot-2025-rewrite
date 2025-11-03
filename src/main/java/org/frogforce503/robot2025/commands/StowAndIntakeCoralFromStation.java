package org.frogforce503.robot2025.commands;

import org.frogforce503.lib.auto.builder.PlannedPathGenerator;
import org.frogforce503.lib.planning.planned_path.PlannedPath;
import org.frogforce503.lib.planning.planned_path.Waypoint;
import org.frogforce503.lib.reefscape.PrescoreBoundary;
import org.frogforce503.lib.reefscape.ReefSide;
import org.frogforce503.lib.util.ProximityUtil;
import org.frogforce503.robot2025.FieldInfo;
import org.frogforce503.robot2025.commands.drive.DrivePlannedPath;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.leds.Leds;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;
import org.frogforce503.robot2025.subsystems.superstructure.arm.Arm;
import org.frogforce503.robot2025.subsystems.superstructure.arm.ArmGoal;
import org.frogforce503.robot2025.subsystems.superstructure.claw.Claw;
import org.frogforce503.robot2025.subsystems.superstructure.claw.ClawGoal;
import org.frogforce503.robot2025.subsystems.superstructure.elevator.Elevator;
import org.frogforce503.robot2025.subsystems.superstructure.elevator.ElevatorGoal;
import org.frogforce503.robot2025.subsystems.superstructure.intake.IntakeGoal;
import org.frogforce503.robot2025.subsystems.superstructure.intake.pivot.IntakePivot;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.Wrist;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.WristGoal;

import edu.wpi.first.wpilibj2.command.Command;

public class StowAndIntakeCoralFromStation extends Command {
    // Requirements
    private final Drive drive;
    private final FieldInfo field;

    private final Superstructure superstructure;
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final Claw claw;
    private final IntakePivot intakePivot;

    private final Leds leds;

    private final PrescoreBoundary prescoreBoundary;

    // State
    private ReefSide closestReefSide;
    private PlannedPath pathToStation;
    private DrivePlannedPath driveToStation;

    private IntakingState currentState = IntakingState.PUT_INTAKEPIVOT_OUT;

    private enum IntakingState {
        SAFE_DISTANCE_FROM_REEF,
        PUT_INTAKEPIVOT_OUT,
        PARTIAL_STOW,
        STOW,
        WAIT_FOR_LOWER_TRUE,
        WAIT_FOR_LOWER_FALSE,
        FINISHED
    }

    public StowAndIntakeCoralFromStation(Drive drive, FieldInfo field, Superstructure superstructure, Leds leds) {
        this.drive = drive;
        this.field = field;

        this.superstructure = superstructure;
        this.elevator = superstructure.getElevator();
        this.arm = superstructure.getArm();
        this.wrist = superstructure.getWrist();
        this.claw = superstructure.getClaw();
        this.intakePivot = superstructure.getIntakePivot();

        this.leds = leds;

        this.prescoreBoundary = new PrescoreBoundary(drive, field);

        addRequirements(drive, elevator, arm, wrist, claw, intakePivot, leds);
    }

    @Override
    public void initialize() {
        if (superstructure.isHasCoral()) {
            currentState = IntakingState.FINISHED;
        }

        closestReefSide = ProximityUtil.getClosestReefSide(drive, field);
        
        pathToStation =
            PlannedPathGenerator.generate(
                0,
                0,
                0,
                0,
                Waypoint.fromHolonomicPose(drive.getCurrentPose()),
                Waypoint.fromHolonomicPose(ProximityUtil.getClosestStation(drive, field).getTarget(field).get()));
        
        driveToStation = new DrivePlannedPath(drive, field, pathToStation);
        driveToStation.initialize();
    }

    @Override
    public void execute() {
        switch (currentState) {
            case SAFE_DISTANCE_FROM_REEF:
                if (!prescoreBoundary.insideBoundary()) {
                    currentState = IntakingState.PUT_INTAKEPIVOT_OUT;
                }
                break;

            case PUT_INTAKEPIVOT_OUT:
                intakePivot.setGoal(IntakeGoal.SCORE_CLEARANCE);
                currentState = IntakingState.PARTIAL_STOW;
                break;

            case PARTIAL_STOW:
                elevator.setGoal(ElevatorGoal.DOWN);
                currentState = IntakingState.STOW;
                break;

            case STOW:
                arm.setGoal(ArmGoal.DOWN);
                wrist.setGoal(WristGoal.INTAKE_CORAL);
                claw.setGoal(ClawGoal.INTAKE_CORAL);
                currentState = IntakingState.WAIT_FOR_LOWER_TRUE;
                break;
                
            case WAIT_FOR_LOWER_TRUE:
                intakePivot.setGoal(IntakeGoal.INTAKE_CLEARANCE);
                if (superstructure.lowerSensorTripped()) {
                    currentState = IntakingState.WAIT_FOR_LOWER_FALSE;
                }
                break;

            case WAIT_FOR_LOWER_FALSE:
                if (!superstructure.lowerSensorTripped() && claw.coralCurrentThresholdForIntookMet()) {
                    currentState = IntakingState.FINISHED;
                }
                break;

            case FINISHED:
                claw.stop();
                superstructure.setHasCoral(true);
                break;    
        }

        driveToStation.execute();
    }

    @Override
    public boolean isFinished() {
        return currentState == IntakingState.FINISHED;
    }

    @Override
    public void end(boolean interrupted) {
        driveToStation.end(interrupted);
    }
}
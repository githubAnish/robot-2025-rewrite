package org.frogforce503.robot2025.commands;

import org.frogforce503.lib.auto.planned_path.PlannedPath;
import org.frogforce503.lib.auto.planned_path.PlannedPathFactory;
import org.frogforce503.lib.auto.planned_path.components.Waypoint;
import org.frogforce503.lib.reefscape.ProximityUtil;
import org.frogforce503.robot2025.commands.drive.DrivePlannedPath;
import org.frogforce503.robot2025.constants.field.FieldConstants;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.leds.Animations;
import org.frogforce503.robot2025.subsystems.leds.Leds;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;
import org.frogforce503.robot2025.subsystems.superstructure.arm.Arm;
import org.frogforce503.robot2025.subsystems.superstructure.arm.ArmConstants;
import org.frogforce503.robot2025.subsystems.superstructure.claw.Claw;
import org.frogforce503.robot2025.subsystems.superstructure.claw.ClawConstants;
import org.frogforce503.robot2025.subsystems.superstructure.elevator.Elevator;
import org.frogforce503.robot2025.subsystems.superstructure.elevator.ElevatorConstants;
import org.frogforce503.robot2025.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot2025.subsystems.superstructure.intakepivot.IntakePivotConstants;
import org.frogforce503.robot2025.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.Wrist;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.WristConstants;
import org.frogforce503.robot2025.subsystems.vision.Vision;
import org.frogforce503.robot2025.subsystems.vision.apriltag_detection.AprilTagGoal;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class SafelyStowAndIntakeCoralFromStation extends Command {
    // Requirements
    private final Drive drive;
    private final Vision vision;

    private final Superstructure superstructure;
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final Claw claw;
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;

    private final Leds leds;
    
    // Constants
    private final double safeDistanceFromReefToStowInches = Units.inchesToMeters(40); // TODO make sure to tune this

    // State
    private Pose2d closestReefSide;
    private DrivePlannedPath driveToStation;

    private IntakingState currentState = IntakingState.SAFE_DISTANCE_FROM_REEF;

    private enum IntakingState {
        SAFE_DISTANCE_FROM_REEF,
        PUT_INTAKEPIVOT_OUT,
        PARTIAL_STOW,
        STOW,
        PUT_INTAKEPIVOT_IN,
        WAIT_FOR_LOWER_TRUE,
        WAIT_FOR_LOWER_FALSE,
        FINISHED
    }

    public SafelyStowAndIntakeCoralFromStation(Drive drive, Vision vision, Superstructure superstructure, Leds leds) {
        this.drive = drive;
        this.vision = vision;

        this.superstructure = superstructure;
        this.elevator = superstructure.getElevator();
        this.arm = superstructure.getArm();
        this.wrist = superstructure.getWrist();
        this.claw = superstructure.getClaw();
        this.intakePivot = superstructure.getIntakePivot();
        this.intakeRoller = superstructure.getIntakeRoller();

        this.leds = leds;

        addRequirements(drive, vision, elevator, arm, wrist, claw, intakePivot, intakeRoller, leds);
    }

    @Override
    public void initialize() {
        // Finishes command if robot already has coral
        if (superstructure.isHasCoral()) {
            currentState = IntakingState.FINISHED;
            return;
        }

        // If elevator & arm close to stowed, no need to move intake pivot
        if (elevator.isAtHeight(ElevatorConstants.minHeight, Units.inchesToMeters(5.0)) &&
            arm.isAtAngle(ArmConstants.STOW, Units.degreesToRadians(5.0))
        ) {
            currentState = IntakingState.PARTIAL_STOW;
        }

        // Generate path to closest station
        closestReefSide = ProximityUtil.getClosestReefSide(drive);

        Pose2d closestStation =
            ProximityUtil.getClosestPose(
                drive,
                FieldConstants.CoralStation.blueLeft,
                FieldConstants.CoralStation.blueRight,
                FieldConstants.CoralStation.redLeft,
                FieldConstants.CoralStation.redRight);
        
        double linearVelocity =
            Math.hypot(
                drive.getRobotVelocity().vxMetersPerSecond,
                drive.getRobotVelocity().vyMetersPerSecond);

        PlannedPath pathToStation =
            PlannedPathFactory.generate(
                3.048,
                3.6576,
                linearVelocity,
                0,
                Waypoint.fromHolonomicPose(drive.getPose()),
                Waypoint.fromHolonomicPose(closestStation));
        
        driveToStation = new DrivePlannedPath(drive, pathToStation);
        driveToStation.schedule();

        vision.setDesiredAprilTagGoal(AprilTagGoal.CORAL_STATION_ALIGNMENT);
        leds.runAnimation(Animations.INTAKE_CORAL);
    }

    @Override
    public void execute() {
        switch (currentState) {
            case SAFE_DISTANCE_FROM_REEF:
                if (ProximityUtil.getDistanceFromPose(drive, closestReefSide) > safeDistanceFromReefToStowInches) {
                    currentState = IntakingState.PUT_INTAKEPIVOT_OUT;
                }
                break;

            case PUT_INTAKEPIVOT_OUT:
                intakePivot.setAngle(IntakePivotConstants.ARM_STOW_CLEARANCE);
                
                if (intakePivot.isAtAngle(IntakePivotConstants.ARM_STOW_CLEARANCE, Units.degreesToRadians(5.0))) { // just needs to be enough out of way for arm
                    currentState = IntakingState.PARTIAL_STOW;
                }
                break;

            case PARTIAL_STOW:
                elevator.setHeight(ElevatorConstants.minHeight);
                currentState = IntakingState.STOW;
                break;

            case STOW:
                arm.setAngle(ArmConstants.STOW);
                wrist.setAngle(WristConstants.INTAKE_CORAL);
                claw.setVelocity(ClawConstants.INTAKE_CORAL);

                if (arm.isAtAngle(ArmConstants.STOW, ArmConstants.kTolerance) && wrist.isAtAngle(WristConstants.INTAKE_CORAL, WristConstants.kTolerance)) {
                    currentState = IntakingState.PUT_INTAKEPIVOT_IN;
                }
                break;

            case PUT_INTAKEPIVOT_IN:
                intakePivot.setAngle(IntakePivotConstants.INTAKE_CLEARANCE);
                currentState = IntakingState.WAIT_FOR_LOWER_TRUE;
                break;
                
            case WAIT_FOR_LOWER_TRUE:
                if (superstructure.lowerBeamTriggered()) {
                    currentState = IntakingState.WAIT_FOR_LOWER_FALSE;
                }
                break;

            case WAIT_FOR_LOWER_FALSE:
                if (!superstructure.lowerBeamTriggered() && claw.coralCurrentThresholdForIntookMet()) {
                    currentState = IntakingState.FINISHED;
                }
                break;

            case FINISHED:
                claw.stop();
                superstructure.setHasCoral(true);
                break;
        }

        Logger.recordOutput("SafelyStowAndIntakeCoralFromStation/State", currentState);
    }

    @Override
    public boolean isFinished() {
        return currentState == IntakingState.FINISHED;
    }

    @Override
    public void end(boolean interrupted) {
        driveToStation.cancel();
        vision.setDesiredAprilTagGoal(AprilTagGoal.GLOBAL_LOCALIZATION);
        leds.stop();
    }
}
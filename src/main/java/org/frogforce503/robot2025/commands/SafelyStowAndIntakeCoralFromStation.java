package org.frogforce503.robot2025.commands;

import org.frogforce503.lib.auto.builder.PlannedPathGenerator;
import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.lib.planning.planned_path.PlannedPath;
import org.frogforce503.lib.planning.planned_path.Waypoint;
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
    private final LoggedTunableNumber safeDistanceFromReefToStow =
        new LoggedTunableNumber("Safe Distance From Reef To Stow Inches", Units.inchesToMeters(40)); // TODO make sure to tune this

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
        if (superstructure.isHasCoral()) {
            currentState = IntakingState.FINISHED;
        }

        // Generate path to closest station
        closestReefSide = ProximityUtil.getClosestReefSide(drive);

        final Pose2d closestStation =
            ProximityUtil.getClosestPose(
                drive,
                FieldConstants.CoralStation.blueLeft,
                FieldConstants.CoralStation.blueRight,
                FieldConstants.CoralStation.redLeft,
                FieldConstants.CoralStation.redRight);
        
        PlannedPath pathToStation =
            PlannedPathGenerator.generate(
                0,
                0,
                0,
                0,
                Waypoint.fromHolonomicPose(drive.getCurrentPose()),
                Waypoint.fromHolonomicPose(closestStation));
        
        driveToStation = new DrivePlannedPath(drive, pathToStation);

        driveToStation.initialize();

        vision.setDesiredAprilTagGoal(AprilTagGoal.CORAL_STATION_ALIGNMENT);
        leds.runAnimation(Animations.INTAKE_CORAL);
    }

    @Override
    public void execute() {
        switch (currentState) {
            case SAFE_DISTANCE_FROM_REEF:
                if (ProximityUtil.getDistanceFromPose(drive, closestReefSide) > safeDistanceFromReefToStow.get()) {
                    currentState = IntakingState.PUT_INTAKEPIVOT_OUT;
                }
                break;

            case PUT_INTAKEPIVOT_OUT:
                intakePivot.setAngle(IntakePivotConstants.SCORE_CLEARANCE);
                
                if (intakePivot.isAtAngle(IntakePivotConstants.SCORE_CLEARANCE, Units.degreesToRadians(5.0))) { // just needs to be enough out of way for arm
                    currentState = IntakingState.PARTIAL_STOW;
                }
                break;

            case PARTIAL_STOW:
                elevator.setHeight(ElevatorConstants.minHeight);
                currentState = IntakingState.STOW;
                break;

            case STOW:
                arm.setAngle(ArmConstants.STOW_ANGLE);
                wrist.setAngle(WristConstants.INTAKE_CORAL);
                claw.setVelocity(ClawConstants.INTAKE_CORAL);

                if (arm.isAtAngle(ArmConstants.STOW_ANGLE, ArmConstants.kTolerance) && wrist.isAtAngle(WristConstants.INTAKE_CORAL, WristConstants.kTolerance)) {
                    currentState = IntakingState.WAIT_FOR_LOWER_TRUE;
                }
                break;

            case PUT_INTAKEPIVOT_IN:
                intakePivot.setAngle(IntakePivotConstants.LOW_CLEARANCE);
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

        driveToStation.execute();
    }

    @Override
    public boolean isFinished() {
        return
            currentState == IntakingState.FINISHED &&
            driveToStation.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        driveToStation.end(interrupted);
        vision.setDesiredAprilTagGoal(AprilTagGoal.GLOBAL_LOCALIZATION);
        leds.stop();
    }
}
package org.frogforce503.robot.commands;

import org.frogforce503.lib.auto.planned_path.PlannedPath;
import org.frogforce503.lib.auto.planned_path.PlannedPath.HolonomicState;
import org.frogforce503.lib.auto.planned_path.PlannedPathFactory;
import org.frogforce503.lib.auto.planned_path.components.Waypoint;
import org.frogforce503.lib.reefscape.ProximityUtil;
import org.frogforce503.lib.swerve.SwervePathController;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.subsystems.leds.Leds;
import org.frogforce503.robot.subsystems.leds.LedsRequest;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.superstructure.arm.Arm;
import org.frogforce503.robot.subsystems.superstructure.arm.ArmConstants;
import org.frogforce503.robot.subsystems.superstructure.claw.Claw;
import org.frogforce503.robot.subsystems.superstructure.claw.ClawConstants;
import org.frogforce503.robot.subsystems.superstructure.elevator.Elevator;
import org.frogforce503.robot.subsystems.superstructure.elevator.ElevatorConstants;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivotConstants;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.wrist.Wrist;
import org.frogforce503.robot.subsystems.superstructure.wrist.WristConstants;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.frogforce503.robot.subsystems.vision.apriltag_detection.AprilTagGoal;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
    
    // Constants
    private final double safeDistanceFromReefToStowInches = Units.inchesToMeters(20); // TODO make sure to tune this

    // Trajectory
    private final SwervePathController trajectoryController = DriveConstants.pathFollower;
    private final Timer trajectoryTimer = new Timer();
    private PlannedPath trajectory;

    // State
    private Pose2d closestReefSide; 
    private Pose2d closestStation;
    private IntakingState currentState = IntakingState.PUT_INTAKEPIVOT_OUT_AND_CHECK_SAFE_DISTANCE_FROM_REEF;

    private enum IntakingState {
        PUT_INTAKEPIVOT_OUT_AND_CHECK_SAFE_DISTANCE_FROM_REEF,
        PARTIAL_STOW,
        STOW,
        PUT_INTAKEPIVOT_IN,
        WAIT_FOR_LOWER_TRUE,
        WAIT_FOR_LOWER_FALSE,
        SET_HAS_CORAL,
        FINISHED
    }

    public SafelyStowAndIntakeCoralFromStation(Drive drive, Vision vision, Superstructure superstructure) {
        this.drive = drive;
        this.vision = vision;

        this.superstructure = superstructure;
        this.elevator = superstructure.getElevator();
        this.arm = superstructure.getArm();
        this.wrist = superstructure.getWrist();
        this.claw = superstructure.getClaw();
        this.intakePivot = superstructure.getIntakePivot();
        this.intakeRoller = superstructure.getIntakeRoller();

        addRequirements(drive, vision, elevator, arm, wrist, claw, intakePivot, intakeRoller);
    }

    @Override
    public void initialize() {
        // Finishes command if robot already has coral
        if (superstructure.isHasCoral()) {
            currentState = IntakingState.FINISHED;
            return;
        }

        vision.setDesiredAprilTagGoal(AprilTagGoal.CORAL_STATION_ALIGNMENT);

        // If elevator & arm close to stowed, no need to move intake pivot
        if (elevator.isAtHeight(ElevatorConstants.minHeight, Units.inchesToMeters(5.0)) &&
            arm.isAtAngle(ArmConstants.STOW, Units.degreesToRadians(5.0))
        ) {
            currentState = IntakingState.PARTIAL_STOW;
        }

        // Generate path to closest station
        closestReefSide = ProximityUtil.getClosestReefSide(drive);
        closestStation = ProximityUtil.getClosestStation(drive);
        
        double linearVelocity =
            Math.hypot(
                drive.getRobotVelocity().vxMetersPerSecond,
                drive.getRobotVelocity().vyMetersPerSecond);

        trajectory =
            PlannedPathFactory.generate(
                3.048,
                3.6576,
                linearVelocity,
                0,
                Waypoint.fromHolonomicPose(drive.getPose()),
                Waypoint.fromHolonomicPose(closestStation));

        trajectoryController.reset();
        trajectoryTimer.restart();
    }

    @Override
    public void execute() {
        switch (currentState) {
            case PUT_INTAKEPIVOT_OUT_AND_CHECK_SAFE_DISTANCE_FROM_REEF:
                intakePivot.setAngle(IntakePivotConstants.ARM_STOW_CLEARANCE);

                if (ProximityUtil.getDistanceFromPose(drive, closestReefSide) > safeDistanceFromReefToStowInches &&
                    intakePivot.isAtAngle(IntakePivotConstants.ARM_STOW_CLEARANCE, Units.degreesToRadians(5.0)) // just needs to be enough out of way for arm
                ) {
                    currentState = IntakingState.PARTIAL_STOW;
                }
                break;

            case PARTIAL_STOW:
                elevator.setHeight(ElevatorConstants.minHeight);

                if (elevator.isAtHeight(ElevatorConstants.minHeight, Units.inchesToMeters(15.0))) {
                    currentState = IntakingState.STOW;
                }
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
                if (RobotBase.isSimulation() && ProximityUtil.getDistanceFromPose(drive, closestStation) < Units.inchesToMeters(25)) {
                    // In sim, we don't have beam breaks, so we just assume intake is successful after intake pivot is in & robot close to station
                    currentState = IntakingState.SET_HAS_CORAL;
                    break;
                }

                if (superstructure.lowerBeamTriggered()) {
                    currentState = IntakingState.WAIT_FOR_LOWER_FALSE;
                }
                break;

            case WAIT_FOR_LOWER_FALSE:
                if (!superstructure.lowerBeamTriggered() && claw.coralCurrentThresholdForIntookMet()) {
                    currentState = IntakingState.SET_HAS_CORAL;
                }
                break;

            case SET_HAS_CORAL:
                claw.stop();
                superstructure.setHasCoral(true);
                currentState = IntakingState.FINISHED;
                break;

            case FINISHED:
                break;
        }

        // Follow trajectory to station
        double currentTime = trajectoryTimer.get();
        HolonomicState desiredState = trajectory.sample(currentTime);
        ChassisSpeeds targetChassisSpeeds = trajectoryController.calculate(drive.getPose(), desiredState);
        drive.runVelocity(targetChassisSpeeds);

        // Log data
        Logger.recordOutput("SafelyStowAndIntakeCoralFromStation/State", currentState);
        Logger.recordOutput("SafelyStowAndIntakeCoralFromStation/Timestamp", currentTime);
        Logger.recordOutput("SafelyStowAndIntakeCoralFromStation/Drive Error", trajectoryController.getPoseError().getTranslation());
        Logger.recordOutput("SafelyStowAndIntakeCoralFromStation/Theta Error", trajectoryController.getRotationError());
    }

    @Override
    public boolean isFinished() {
        return currentState == IntakingState.FINISHED;
    }

    @Override
    public void end(boolean interrupted) {
        trajectoryTimer.stop();
        drive.stop();
        vision.setDesiredAprilTagGoal(AprilTagGoal.GLOBAL_LOCALIZATION);
    }
}
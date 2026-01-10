package org.frogforce503.robot.commands;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.lib.reefscape.ProximityUtil;
import org.frogforce503.robot.commands.drive.DriveToPose;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.leds.Leds;
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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAlgaeFromReef extends Command { // use the reef faces from FieldConstants.Reef, basically this command should do it, wait until algae fully in, then backup
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
    private final Transform2d reefSideToIntake = GeomUtil.toTransform2d(Units.inchesToMeters(12.5), 0);
    private final Transform2d reefSideToBackup = GeomUtil.toTransform2d(Units.inchesToMeters(18.5), 0);

    // State
    private boolean highAlgae; // based off of tag ID / reef face

    private DriveToPose driveToAlgae;
    private DriveToPose backupFromAlgae;

    private IntakingState currentState = IntakingState.PUT_INTAKEPIVOT_OUT;

    private enum IntakingState {
        PUT_INTAKEPIVOT_OUT,
        RAISE_FOR_ALGAE,
        PUT_INTAKEPIVOT_IN,
        DRIVE_TO_REEF_AND_INTAKE_ALGAE,
        BACK_UP_WITH_ALGAE,
        STOW,
        FINISHED
    }
    
    public IntakeAlgaeFromReef(Drive drive, Vision vision, Superstructure superstructure) {
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
        // Finishes command if robot already has a gamepiece
        if (superstructure.isHasCoral() || superstructure.isHasAlgaeInClaw() || superstructure.isHasAlgaeInIntake()) {
            currentState = IntakingState.FINISHED;
            return;
        }

        final Pose2d closestReefSide = ProximityUtil.getClosestReefSide(drive);

        highAlgae =
            closestReefSide.equals(FieldConstants.Reef.blueFaceCenters[0]) ||
            closestReefSide.equals(FieldConstants.Reef.blueFaceCenters[2]) ||
            closestReefSide.equals(FieldConstants.Reef.blueFaceCenters[4]) ||
            closestReefSide.equals(FieldConstants.Reef.redFaceCenters[0]) ||
            closestReefSide.equals(FieldConstants.Reef.redFaceCenters[2]) ||
            closestReefSide.equals(FieldConstants.Reef.redFaceCenters[4]);

        driveToAlgae = new DriveToPose(drive, () -> closestReefSide.plus(reefSideToIntake));
        backupFromAlgae = new DriveToPose(drive, () -> closestReefSide.plus(reefSideToBackup));

        vision.setDesiredAprilTagGoal(AprilTagGoal.REEF_ALIGNMENT);
    }

    @Override
    public void execute() {
        switch (currentState) {
            case PUT_INTAKEPIVOT_OUT:
                intakePivot.setAngle(IntakePivotConstants.ARM_STOW_CLEARANCE);
                
                if (intakePivot.isAtAngle(IntakePivotConstants.ARM_STOW_CLEARANCE, Units.degreesToRadians(5.0))) { // just needs to be enough out of way for arm
                    currentState = IntakingState.RAISE_FOR_ALGAE;
                }
                break;

            case RAISE_FOR_ALGAE:
                double elevatorSetpoint = highAlgae ? ElevatorConstants.ALGAE_PLUCK_HIGH : ElevatorConstants.ALGAE_PLUCK_LOW;
                double armSetpoint = highAlgae ? ArmConstants.ALGAE_PLUCK_HIGH : ArmConstants.ALGAE_PLUCK_LOW;
                double wristSetpoint = highAlgae ? WristConstants.ALGAE_PLUCK_HIGH : WristConstants.ALGAE_PLUCK_LOW;
                double clawSetpoint = highAlgae ? ClawConstants.ALGAE_PLUCK_HIGH : ClawConstants.ALGAE_PLUCK_LOW;

                elevator.setHeight(elevatorSetpoint);
                arm.setAngle(armSetpoint);
                wrist.setAngle(wristSetpoint);
                claw.setVelocity(clawSetpoint);
                
                if (arm.isAtAngle(armSetpoint, ArmConstants.kTolerance) && wrist.isAtAngle(wristSetpoint, WristConstants.kTolerance)) {
                    currentState = IntakingState.PUT_INTAKEPIVOT_IN;
                }
                break;

            case PUT_INTAKEPIVOT_IN:
                intakePivot.setAngle(IntakePivotConstants.INTAKE_CLEARANCE);
                currentState = IntakingState.DRIVE_TO_REEF_AND_INTAKE_ALGAE;
                break;

            case DRIVE_TO_REEF_AND_INTAKE_ALGAE:
                driveToAlgae.schedule();

                if (claw.algaeCurrentThresholdForHoldMet()) {
                    superstructure.setHasAlgaeInClaw(true);
                    driveToAlgae.cancel();
                    currentState = IntakingState.BACK_UP_WITH_ALGAE;
                }
                break;

            case BACK_UP_WITH_ALGAE:
                
                break;

            case STOW:
                break;

            case FINISHED:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return currentState == IntakingState.FINISHED;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
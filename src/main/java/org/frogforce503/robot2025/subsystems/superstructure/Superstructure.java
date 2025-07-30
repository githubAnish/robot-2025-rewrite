package org.frogforce503.robot2025.subsystems.superstructure;

import java.util.function.Supplier;

import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.lib.util.Logic;
import org.frogforce503.robot2025.commands.coral_score_reef.Branch;
import org.frogforce503.robot2025.subsystems.superstructure.arm.Arm;
import org.frogforce503.robot2025.subsystems.superstructure.arm.Arm.ArmGoal;
import org.frogforce503.robot2025.subsystems.superstructure.claw.Claw;
import org.frogforce503.robot2025.subsystems.superstructure.claw.Claw.ClawGoal;
import org.frogforce503.robot2025.subsystems.superstructure.elevator.Elevator;
import org.frogforce503.robot2025.subsystems.superstructure.elevator.Elevator.ElevatorGoal;
import org.frogforce503.robot2025.subsystems.superstructure.intake.Intake;
import org.frogforce503.robot2025.subsystems.superstructure.intake.Intake.IntakeGoal;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.CoralSensorIO;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.CoralSensorIOInputsAutoLogged;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.Wrist;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.Wrist.WristGoal;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

public class Superstructure extends SubsystemBase implements SuperstructureBaseFactory {
    // Subsystems
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final Claw claw;
    private final Intake intake;

    private final CoralSensorIO coralSensorIO;
    private final CoralSensorIOInputsAutoLogged coralSensorInputs = new CoralSensorIOInputsAutoLogged();

    // Inputs
    @Setter @Getter private boolean hasCoral;
    @Setter @Getter private boolean hasAlgaeInClaw;
    @Setter @Getter private boolean hasAlgaeInIntake;

    @Setter @Getter private Mode currentMode = Mode.CORAL_INTAKE;
    @Setter @Getter private Gamepiece currentPiece = Gamepiece.CORAL;
    @Setter @Getter private Branch currentBranch = Branch.LEFT;

    // Visualizers
    @Getter private final SuperstructureVisualizer measuredVisualizer, setpointVisualizer;

    // Overrides
    @Getter private boolean manualControlEnabled;
    @Getter private boolean brakeModeEnabled;

    public Superstructure(Elevator elevator, Arm arm, Wrist wrist, Claw claw, Intake intake, CoralSensorIO coralSensorIO, Supplier<Pose2d> robotPoseSupplier) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        this.intake = intake;
        this.coralSensorIO = coralSensorIO;

        this.measuredVisualizer = new SuperstructureVisualizer("Measured", claw, robotPoseSupplier);
        this.setpointVisualizer = new SuperstructureVisualizer("Setpoint", claw, robotPoseSupplier);
    }

    @Override
    public void periodic() {
        coralSensorIO.updateInputs(coralSensorInputs);
        Logger.processInputs("CoralSensors", coralSensorInputs);

        // Update visualizers
        measuredVisualizer.updateOnlyIfInSimulation(
            elevator.getPosition(),
            arm.getPosition(),
            wrist.getPosition(),
            intake.getPivotPosition());

        setpointVisualizer.updateOnlyIfInSimulation(
            elevator.getSetpoint().position,
            arm.getSetpoint().position,
            wrist.getSetpoint().position,
            intake.getSetpoint().position);

        Logger.recordOutput("Superstructure/Brake Mode Enabled", brakeModeEnabled);

        Logger.recordOutput("Superstructure/Inputs/Has Coral", hasCoral);
        Logger.recordOutput("Superstructure/Inputs/Algae In Claw", hasAlgaeInClaw);
        Logger.recordOutput("Superstructure/Inputs/Algae In Intake", hasAlgaeInIntake);

        Logger.recordOutput("Superstructure/Current Gamepiece", currentPiece);
        Logger.recordOutput("Superstructure/Mode", currentMode);
        Logger.recordOutput("Superstructure/Branch", currentBranch);

        // Record cycle time
        LoggedTracer.record("Superstructure");
    }

    public boolean upperSensorGot() {
        return coralSensorInputs.data.upperGot();
    }

    public boolean lowerSensorGot() {
        return coralSensorInputs.data.lowerGot();
    }

    public void setPiece() {
        currentPiece =
            currentPiece == Gamepiece.CORAL
                ? Gamepiece.ALGAE
                : Gamepiece.CORAL;
    }

    public void seedWristPosition() {
        if (MathUtils.inRange(arm.getPosition(), 0, 30) &&
            MathUtils.inRange(wrist.getAbsolutePosition(), 0, 180)
        ) {
            wrist.setEncoderPosition(
                arm.getPosition() + wrist.getAbsolutePosition());
        }
    }

    /** Set the superstructure brake mode only if no subsystem has brake mode set individually through each subsystem's coast override. */
    public void setBrakeMode(boolean enabled) {
        brakeModeEnabled = enabled;

        elevator.setBrakeMode(enabled);
        arm.setBrakeMode(enabled);
        wrist.setBrakeMode(enabled);
        claw.setBrakeMode(enabled);
        intake.setBrakeMode(enabled);
    }

    // Manual Control
    public void toggleManualControl() {
        manualControlEnabled = !manualControlEnabled;
    }

    public Command manualElevatorControl(double percent) {
        return elevator.runManual(percent);
    }

    public Command manualArmControl(double percent) {
        return arm.runManual(percent);
    }

    public Command manualWristControl(double percent) {
        return wrist.runManual(percent);
    }

    public Command manualIntakeControl(double percent) {
        return intake.runManual(percent);
    }

    // Climbing Commands
    public Command setPivotDown() {
        return manualIntakeControl(-0.2);
    }

    public Command bringPivotUp() {
        return manualIntakeControl(0.2);
    }

    // Main Commands
    public Command stop() {
        return
            Commands.parallel(
                elevator.stop(),
                arm.stop(),
                wrist.stop(),
                claw.stop(),
                intake.stop());
    }

    @Override
    public Command intakeCoral() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.SCORE_CLEARANCE),
                Commands.parallel(
                    elevator.runGoal(ElevatorGoal.DOWN),
                    arm.runGoal(ArmGoal.DOWN),
                    wrist.runGoal(WristGoal.INTAKE_CORAL),
                    claw.runGoal(ClawGoal.INTAKE_CORAL)
                ),
                intake.runGoal(IntakeGoal.INTAKE_CLEARANCE),
                Commands.waitUntil(this::lowerSensorGot),
                Commands.waitUntil(
                    Logic.and(
                        Logic.not(this::lowerSensorGot),
                        claw::coralCurrentThresholdForIntookMet)),
                claw.stop(),
                Commands.runOnce(() -> hasCoral = true)
            )
            .onlyIf(() -> !hasCoral);
    }

    @Override
    public Command preScoreL1() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.SCORE_CLEARANCE),
                Commands.parallel(
                    elevator.runGoal(ElevatorGoal.PRESCORE_L1),
                    arm.runGoal(ArmGoal.PRESCORE_L1),
                    wrist.runGoal(WristGoal.PRESCORE_L1)
                ),
                intake.runGoal(IntakeGoal.INTAKE_CLEARANCE)
            );
    }

    @Override
    public Command preScoreL2() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.SCORE_CLEARANCE),
                Commands.parallel(
                    elevator.runGoal(ElevatorGoal.PRESCORE_L2),
                    arm.runGoal(ArmGoal.PRESCORE_L2),
                    wrist.runGoal(WristGoal.PRESCORE_L2)
                ),
                intake.runGoal(IntakeGoal.INTAKE_CLEARANCE)
            );
    }

    @Override
    public Command preScoreL3() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.SCORE_CLEARANCE),
                Commands.parallel(
                    elevator.runGoal(ElevatorGoal.PRESCORE_L3),
                    arm.runGoal(ArmGoal.PRESCORE_L3),
                    wrist.runGoal(WristGoal.PRESCORE_L3)
                ),
                intake.runGoal(IntakeGoal.INTAKE_CLEARANCE)
            );
    }

    @Override
    public Command preScoreL4() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.SCORE_CLEARANCE),
                Commands.parallel(
                    elevator.runGoal(ElevatorGoal.PRESCORE_L4),
                    arm.runGoal(ArmGoal.PRESCORE_L4),
                    wrist.runGoal(WristGoal.PRESCORE_L4)
                ),
                intake.runGoal(IntakeGoal.INTAKE_CLEARANCE)
            );
    }

    @Override
    public Command scoreL1() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.SCORE_CLEARANCE),
                Commands.parallel(
                    arm.runGoal(ArmGoal.SCORE_L1),
                    wrist.runGoal(WristGoal.SCORE_L1)
                ),
                elevator.runGoal(ElevatorGoal.SCORE_L1),
                intake.runGoal(IntakeGoal.INTAKE_CLEARANCE)
            );
    }

    @Override
    public Command scoreL2() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.SCORE_CLEARANCE),
                Commands.parallel(
                    arm.runGoal(ArmGoal.SCORE_L2),
                    wrist.runGoal(WristGoal.SCORE_L2)
                ),
                elevator.runGoal(ElevatorGoal.SCORE_L2),
                intake.runGoal(IntakeGoal.INTAKE_CLEARANCE)
            );
    }

    @Override
    public Command scoreL3() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.SCORE_CLEARANCE),
                Commands.parallel(
                    arm.runGoal(ArmGoal.SCORE_L3),
                    wrist.runGoal(WristGoal.SCORE_L3)
                ),
                elevator.runGoal(ElevatorGoal.SCORE_L3),
                intake.runGoal(IntakeGoal.INTAKE_CLEARANCE)
            );
    }

    @Override
    public Command scoreL4() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.SCORE_CLEARANCE),
                Commands.parallel(
                    arm.runGoal(ArmGoal.SCORE_L4),
                    wrist.runGoal(WristGoal.SCORE_L4)
                ),
                elevator.runGoal(ElevatorGoal.SCORE_L4),
                intake.runGoal(IntakeGoal.INTAKE_CLEARANCE)
            );
    }

    @Override
    public Command ejectCoral() {
        return
            Commands.sequence(
                claw.runGoal(ClawGoal.EJECT_CORAL),
                Commands.runOnce(() -> hasCoral = false)
            );
    }

    @Override
    public Command ejectCoralForL1() {
        return
            Commands.sequence(
                claw.runGoal(ClawGoal.EJECT_CORAL_FOR_L1),
                Commands.runOnce(() -> hasCoral = false)
            );
    }

    @Override
    public Command intakeAlgaeFromGround() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.INTAKE_ALGAE_FROM_GROUND)
            )
            .onlyIf(() -> !hasAlgaeInIntake);
    }

    @Override
    public Command holdAlgaeFromGround() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.HOLD_ALGAE)
            );
    }

    @Override
    public Command intakeAlgaeFromHandoff() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.INTAKE_ALGAE_FROM_GROUND),
                Commands.parallel(
                    arm.runGoal(ArmGoal.HANDOFF),
                    wrist.runGoal(WristGoal.HANDOFF),
                    claw.runGoal(ClawGoal.INTAKE_ALGAE)
                )
            );
    }

    @Override
    public Command holdAlgaeFromHandoff() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.HANDOFF_RELEASE),
                Commands.parallel(
                    arm.runGoal(ArmGoal.HANDOFF_RELEASE),
                    wrist.runGoal(WristGoal.HANDOFF_RELEASE)
                ),
                intake.runGoal(IntakeGoal.HANDOFF_EJECT),
                Commands.waitUntil(claw::algaeCurrentThresholdForHoldMet),
                Commands.runOnce(() -> hasAlgaeInClaw = true),
                intake.runGoal(IntakeGoal.INTAKE_CLEARANCE),
                Commands.parallel(
                    claw.runGoal(ClawGoal.HOLD_ALGAE),
                    wrist.runGoal(WristGoal.HOLD_ALGAE)
                )
            );
    }

    @Override
    public Command pluckHighAlgae() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.SCORE_CLEARANCE),
                Commands.parallel(
                    elevator.runGoal(ElevatorGoal.PLUCK_ALGAE_HIGH),
                    arm.runGoal(ArmGoal.PLUCK_ALGAE_HIGH),
                    wrist.runGoal(WristGoal.PLUCK_ALGAE_HIGH),
                    claw.runGoal(ClawGoal.INTAKE_ALGAE)
                ),
                intake.runGoal(IntakeGoal.INTAKE_CLEARANCE)
            );
    }

    @Override
    public Command pluckLowAlgae() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.SCORE_CLEARANCE),
                Commands.parallel(
                    elevator.runGoal(ElevatorGoal.PLUCK_ALGAE_LOW),
                    arm.runGoal(ArmGoal.PLUCK_ALGAE_LOW),
                    wrist.runGoal(WristGoal.PLUCK_ALGAE_LOW),
                    claw.runGoal(ClawGoal.INTAKE_ALGAE)
                ),
                intake.runGoal(IntakeGoal.INTAKE_CLEARANCE)
            );
    }

    @Override
    public Command holdAlgaeFromPluck() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.INTAKE_CLEARANCE),
                elevator.runGoal(ElevatorGoal.DOWN),
                Commands.parallel(
                    arm.runGoal(ArmGoal.HOLD_ALGAE),
                    wrist.runGoal(WristGoal.HOLD_ALGAE),
                    claw.runGoal(ClawGoal.HOLD_ALGAE)
                )
            );
    }

    @Override
    public Command scoreProcessor() {
        return
            Commands.either(
                scoreProcessorFromIntake(),
                scoreProcessorFromClaw(),
                () -> hasAlgaeInIntake)
            .withName("scoreProcessor");
    }

    @Override
    public Command scoreProcessorFromIntake() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.PROCESSOR_EJECT_ALGAE)
            );
    }

    @Override
    public Command scoreProcessorFromClaw() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.SCORE_CLEARANCE),
                Commands.parallel(
                    arm.runGoal(ArmGoal.PROCESSOR_FROM_CLAW),
                    wrist.runGoal(WristGoal.PROCESSOR_FROM_CLAW)
                ),
                intake.runGoal(IntakeGoal.INTAKE_CLEARANCE),
                claw.runGoal(ClawGoal.EJECT_ALGAE)
            );
    }

    @Override
    public Command scoreBarge() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.SCORE_CLEARANCE),
                elevator.runGoal(ElevatorGoal.BARGE),
                Commands.parallel(
                    arm.runGoal(ArmGoal.BARGE),
                    wrist.runGoal(WristGoal.BARGE)
                ),
                intake.runGoal(IntakeGoal.INTAKE_CLEARANCE)
            );
    }

    @Override
    public Command ejectAlgaeFromIntake() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.PROCESSOR_EJECT_ALGAE),
                Commands.runOnce(() -> hasAlgaeInIntake = false)
            );
    }

    @Override
    public Command ejectAlgaeFromClaw() {
        return
            Commands.sequence(
                claw.runGoal(ClawGoal.EJECT_ALGAE),
                Commands.runOnce(() -> hasAlgaeInClaw = false)
            );
    }

    @Override
    public Command home() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.SCORE_CLEARANCE),
                elevator.runGoal(ElevatorGoal.DOWN),
                Commands.parallel(
                    arm.runGoal(ArmGoal.DOWN),
                    wrist.runGoal(WristGoal.INTAKE_CORAL),
                    claw.stop()
                ),
                intake.runGoal(IntakeGoal.INTAKE_CLEARANCE)
            );
    }

    @Override
    public Command homeAfterL4() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.SCORE_CLEARANCE),
                Commands.parallel(
                    elevator.runGoal(ElevatorGoal.DOWN),
                    arm.runGoal(ArmGoal.POSTSCORE_L4),
                    wrist.runGoal(WristGoal.POSTSCORE_L4),
                    claw.stop()
                ),
                intake.runGoal(IntakeGoal.INTAKE_CLEARANCE)
            );
    }

    // Auton-specific commands here
    public Command intakeCoralDuringAuton() {
        return
            Commands.sequence(
            intake.runGoal(IntakeGoal.SCORE_CLEARANCE),
            Commands.parallel(
                elevator.runGoal(ElevatorGoal.DOWN),
                arm.runGoal(ArmGoal.DOWN),
                wrist.runGoal(WristGoal.INTAKE_CORAL),
                claw.runGoal(ClawGoal.INTAKE_CORAL)
            ),
            intake.runGoal(IntakeGoal.INTAKE_CLEARANCE),
            Commands.waitUntil(this::lowerSensorGot),
            Commands.waitUntil(
                Logic.and(
                    Logic.not(this::lowerSensorGot),
                    claw::coralCurrentThresholdForIntookMet)),
            claw.stop(),
            Commands.runOnce(() -> hasCoral = true)
        );
    }

    // Miscellaneous commands here
    public Command stopClaw() {
        return claw.stop();
    }

    public enum Mode {
        CORAL_INTAKE,
        L1,
        L2,
        L3,
        L4,

        ALGAE_GROUND,
        ALGAE_HANDOFF,
        ALGAE_PLUCK_HIGH,
        ALGAE_PLUCK_LOW,
        PROCESSOR,
        BARGE
    }

    public enum Gamepiece {
        CORAL,
        ALGAE
    }
}
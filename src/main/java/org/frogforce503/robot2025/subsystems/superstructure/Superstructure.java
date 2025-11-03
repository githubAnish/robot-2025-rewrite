package org.frogforce503.robot2025.subsystems.superstructure;

import java.util.function.Supplier;

import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.reefscape.Gamepiece;
import org.frogforce503.lib.subsystem.VirtualSubsystem;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.lib.util.Logic;
import org.frogforce503.robot2025.subsystems.superstructure.arm.Arm;
import org.frogforce503.robot2025.subsystems.superstructure.arm.ArmGoal;
import org.frogforce503.robot2025.subsystems.superstructure.claw.Claw;
import org.frogforce503.robot2025.subsystems.superstructure.claw.ClawGoal;
import org.frogforce503.robot2025.subsystems.superstructure.elevator.Elevator;
import org.frogforce503.robot2025.subsystems.superstructure.elevator.ElevatorGoal;
import org.frogforce503.robot2025.subsystems.superstructure.intake.IntakeGoal;
import org.frogforce503.robot2025.subsystems.superstructure.intake.pivot.IntakePivot;
import org.frogforce503.robot2025.subsystems.superstructure.intake.roller.IntakeRoller;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.CoralSensorIO;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.CoralSensorIOInputsAutoLogged;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.Wrist;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.WristGoal;
import org.frogforce503.robot2025.visualization.SuperstructureVisualizer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

public class Superstructure extends VirtualSubsystem {
    // Subsystems
    @Getter private final Elevator elevator;
    @Getter private final Arm arm;
    @Getter private final Wrist wrist;
    @Getter private final Claw claw;
    @Getter private final IntakePivot intakePivot;
    @Getter private final IntakeRoller intakeRoller;

    private final CoralSensorIO coralSensorIO;
    private final CoralSensorIOInputsAutoLogged coralSensorInputs = new CoralSensorIOInputsAutoLogged();

    // Inputs
    @Setter @Getter private boolean hasCoral;
    @Setter @Getter private boolean hasAlgaeInClaw;
    @Setter @Getter private boolean hasAlgaeInIntake;

    @Setter @Getter private SuperstructureMode currentMode = SuperstructureMode.CORAL_INTAKE;
    @Getter private Gamepiece currentPiece = Gamepiece.CORAL;

    // Visualizers
    @Getter private final SuperstructureVisualizer visualizer;

    // Overrides
    private LoggedNetworkBoolean superstructureCoastOverride =
        new LoggedNetworkBoolean("Coast Mode/Superstructure", false);

    public Superstructure(
        Elevator elevator,
        Arm arm,
        Wrist wrist,
        Claw claw,
        IntakePivot intakePivot,
        IntakeRoller intakeRoller,
        CoralSensorIO coralSensorIO,
        Supplier<Pose2d> robotPoseSupplier
    ) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.coralSensorIO = coralSensorIO;

        this.visualizer = new SuperstructureVisualizer(this, robotPoseSupplier);
    }

    @Override
    public void periodic() {
        coralSensorIO.updateInputs(coralSensorInputs);
        Logger.processInputs("CoralSensors", coralSensorInputs);

        setCoastMode(
            DriverStation.isDisabled() &&
            superstructureCoastOverride.get());

        currentPiece = currentMode.getGamepiece();

        // Update visualizers
        if (RobotBase.isSimulation()) {
            visualizer.update(
                elevator.getHeight(),
                arm.getAngle(),
                wrist.getRelativeAngle(),
                intakePivot.getAngle());
        }

        Logger.recordOutput("Superstructure/Has Coral", hasCoral);
        Logger.recordOutput("Superstructure/Algae In Claw", hasAlgaeInClaw);
        Logger.recordOutput("Superstructure/Algae In Intake", hasAlgaeInIntake);

        Logger.recordOutput("Superstructure/Current Gamepiece", currentPiece);
        Logger.recordOutput("Superstructure/Mode", currentMode);

        // Record cycle time
        LoggedTracer.record("Superstructure");
    }

    public boolean upperSensorTripped() {
        return coralSensorInputs.data.upperTripped();
    }

    public boolean lowerSensorTripped() {
        return coralSensorInputs.data.lowerTripped();
    }

    public void setCoastMode(boolean enabled) {
        elevator.getCoastOverride().set(enabled);
        arm.getCoastOverride().set(enabled);
        wrist.getCoastOverride().set(enabled);
        claw.getCoastOverride().set(enabled);
        intakePivot.getCoastOverride().set(enabled);
        intakeRoller.getCoastOverride().set(enabled);
    }

    public void seedWristPosition() {
        if (RobotBase.isReal() &&
            MathUtils.inRange(arm.getAngle(), 0, 30) &&
            MathUtils.inRange(wrist.getAbsoluteAngle(), 0, 180)
        ) {
            wrist.setEncoderPosition(
                arm.getAngle() + wrist.getAbsoluteAngle());
        }
    }

    // Main Commands
    public void stop() {
        elevator.stop();
        arm.stop();
        wrist.stop();
        claw.stop();
        intakePivot.stop();
        intakeRoller.stop();
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
                intake.runGoal(IntakeGoal.INTAKE_ALGAE_FROM_GROUND),
                Commands.runOnce(() -> hasAlgaeInIntake = true)
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
                ),
                Commands.waitUntil(intake::algaeCurrentThresholdForHoldMet),
                Commands.runOnce(() -> hasAlgaeInIntake = true)
            );
    }

    @Override
    public Command holdAlgaeFromHandoff() {
        return
            Commands.sequence(
                intake.runGoal(IntakeGoal.HANDOFF_RELEASE),
                Commands.runOnce(() -> hasAlgaeInIntake = false),
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
                intake.runGoal(IntakeGoal.PROCESSOR_EJECT_ALGAE));
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
}
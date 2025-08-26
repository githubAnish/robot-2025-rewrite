package org.frogforce503.robot2025.subsystems.leds;

import org.frogforce503.lib.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Setter;

public class Leds extends SubsystemBase {
    private final LedsIO io;
    private final LedsIOInputsAutoLogged inputs = new LedsIOInputsAutoLogged();

    @Setter public LedsGoal currentGoal = LedsGoal.OFF;

    @Setter private boolean cameraDisconnected = false;

    public Leds(LedsIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Leds", inputs);

        if (cameraDisconnected) {
            currentGoal = LedsGoal.CAMERA_DISCONNECTED;
        }

        currentGoal.getAction().accept(io);

        Logger.recordOutput("Leds/Goal", currentGoal.name());

        // Record cycle time
        LoggedTracer.record("Leds");
    }

    // Base Commands
    private Command runGoal(LedsGoal goal) {
        return
            runOnce(
                () -> setCurrentGoal(goal))
                    .ignoringDisable(true);
    }

    private Command runCoralGoalTimed(LedsGoal goal, double timeToRunGoalSeconds) {
        return
            Commands.sequence(
                Commands.runOnce(() -> setCurrentGoal(goal)),
                Commands.waitSeconds(timeToRunGoalSeconds),
                Commands.runOnce(() -> setCurrentGoal(LedsGoal.NEUTRAL_CORAL))
            )
            .ignoringDisable(true);
    }

    private Command runAlgaeGoalTimed(LedsGoal goal, double timeToRunGoalSeconds) {
        return
            Commands.sequence(
                Commands.runOnce(() -> setCurrentGoal(goal)),
                Commands.waitSeconds(timeToRunGoalSeconds),
                Commands.runOnce(() -> setCurrentGoal(LedsGoal.NEUTRAL_ALGAE))
            )
            .ignoringDisable(true);
    }

    private Command runCoralGoalUntilCancel(LedsGoal goal) {
        return
            startEnd(
                () -> setCurrentGoal(goal),
                () -> setCurrentGoal(LedsGoal.NEUTRAL_CORAL)
            )
            .ignoringDisable(true);
    }

    private Command runAlgaeGoalUntilCancel(LedsGoal goal) {
        return
            startEnd(
                () -> setCurrentGoal(goal),
                () -> setCurrentGoal(LedsGoal.NEUTRAL_ALGAE)
            )
            .ignoringDisable(true);
    }

    // Commands
    public Command off() {
        return runGoal(LedsGoal.OFF);
    }

    public Command signalCoralMode() {
        return runGoal(LedsGoal.NEUTRAL_CORAL);
    }

    public Command signalAlgaeMode() {
        return runGoal(LedsGoal.NEUTRAL_ALGAE);
    }

    public Command intakeCoral() {
        return runCoralGoalUntilCancel(LedsGoal.INTAKE_CORAL);
    }

    public Command gotCoral() {
        return runCoralGoalTimed(LedsGoal.GOT_CORAL, 3);
    }

    public Command scoreCoral() {
        return runCoralGoalUntilCancel(LedsGoal.SCORE_CORAL);
    }

    public Command intakeAlgae() {
        return runAlgaeGoalUntilCancel(LedsGoal.INTAKE_ALGAE);
    }

    public Command gotAlgae() {
        return runAlgaeGoalTimed(LedsGoal.GOT_ALGAE, 3);
    }

    public Command scoreAlgae() {
        return runAlgaeGoalUntilCancel(LedsGoal.SCORE_ALGAE);
    }

    public Command usingGlobalPose() {
        return runGoal(LedsGoal.GLOBAL_POSE_USED);
    }
}
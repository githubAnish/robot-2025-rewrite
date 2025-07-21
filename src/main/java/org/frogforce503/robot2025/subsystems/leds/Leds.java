package org.frogforce503.robot2025.subsystems.leds;

import java.util.function.Consumer;

import org.frogforce503.lib.leds.Animations;
import org.frogforce503.lib.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Setter;

public class Leds extends SubsystemBase {
    private final LedsIO io;
    private final LedsIOInputsAutoLogged inputs = new LedsIOInputsAutoLogged();

    @Setter public LedsGoal currentGoal = LedsGoal.OFF;

    @Setter private boolean cameraDisconnected = false;

    // Goals
    public enum LedsGoal {
        OFF(io -> io.stop()),

        NEUTRAL_CORAL(io -> io.runAnimation(Animations.BREATHE_PURPLE.getAnimation())),
        INTAKE_CORAL(io -> io.runAnimation(Animations.FLASH_PURPLE.getAnimation())),
        GOT_CORAL(io -> io.runAnimation(Animations.FLASH_GREEN.getAnimation())),
        SCORE_CORAL(io -> io.runAnimation(Animations.FLASH_GREEN.getAnimation())),

        NEUTRAL_ALGAE(io -> io.runAnimation(Animations.BREATHE_BLUE.getAnimation())),
        INTAKE_ALGAE(io -> io.runAnimation(Animations.FLASH_BLUE.getAnimation())),
        GOT_ALGAE(io -> io.runAnimation(Animations.FLASH_GREEN.getAnimation())),
        SCORE_ALGAE(io -> io.runAnimation(Animations.FLASH_GREEN.getAnimation())),

        GLOBAL_POSE_USED(io -> io.runAnimation(Animations.FLASH_RED.getAnimation())),
        CAMERA_DISCONNECTED(io -> io.runAnimation(Animations.FLASH_RED.getAnimation()));
    
        private Consumer<LedsIO> action;
    
        private LedsGoal(Consumer<LedsIO> action) {
            this.action = action;
        }
    }

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

        currentGoal.action.accept(io);

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

    private Command runCoralGoal(LedsGoal goal) {
        return
            startEnd(
                () -> setCurrentGoal(goal),
                () -> setCurrentGoal(LedsGoal.NEUTRAL_CORAL))
                    .ignoringDisable(true);
    }

    private Command runAlgaeGoal(LedsGoal goal) {
        return
            startEnd(
                () -> setCurrentGoal(goal),
                () -> setCurrentGoal(LedsGoal.NEUTRAL_ALGAE))
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
        return runCoralGoal(LedsGoal.INTAKE_CORAL);
    }

    public Command gotCoral() {
        return runCoralGoal(LedsGoal.GOT_CORAL);
    }

    public Command scoreCoral() {
        return runCoralGoal(LedsGoal.SCORE_CORAL);
    }

    public Command intakeAlgae() {
        return runAlgaeGoal(LedsGoal.INTAKE_ALGAE);
    }

    public Command gotAlgae() {
        return runAlgaeGoal(LedsGoal.GOT_ALGAE);
    }

    public Command scoreAlgae() {
        return runAlgaeGoal(LedsGoal.SCORE_ALGAE);
    }

    public Command usingGlobalPose() {
        return runGoal(LedsGoal.GLOBAL_POSE_USED);
    }
}
package org.frogforce503.robot2025.subsystems.leds;

import org.frogforce503.lib.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

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
}
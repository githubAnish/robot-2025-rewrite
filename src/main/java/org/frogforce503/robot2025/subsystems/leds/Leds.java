package org.frogforce503.robot2025.subsystems.leds;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.logging.LoggerUtil;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Setter;

public class Leds extends SubsystemBase {
    private final LedsIO io;
    private final LedsIOInputsAutoLogged inputs = new LedsIOInputsAutoLogged();

    @Setter private boolean cameraDisconnected = false;

    public Leds(LedsIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        LoggerUtil.recordCurrentCommand(this);

        io.updateInputs(inputs);
        Logger.processInputs("Leds", inputs);

        if (cameraDisconnected) {
            io.runAnimation(Animations.CAMERA_DISCONNECTED);
        }

        // Record cycle time
        LoggedTracer.record("Leds");
    }

    public void stop() {
        if (!cameraDisconnected) {
            io.stop();
        }
    }

    public void runColor(Color color) {
        if (!cameraDisconnected) {
            io.runColor(color);
        }
    }

    public void runAnimation(Animation animation) {
        if (!cameraDisconnected) {
            io.runAnimation(animation);
        }
    }
}
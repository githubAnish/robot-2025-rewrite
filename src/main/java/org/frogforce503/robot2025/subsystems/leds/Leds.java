package org.frogforce503.robot2025.subsystems.leds;

import org.frogforce503.lib.logging.LoggerUtil;
import org.frogforce503.lib.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Setter;

public class Leds extends SubsystemBase {
    private final LedsIO io;
    private final LedsIOInputsAutoLogged inputs = new LedsIOInputsAutoLogged();

    // Control
    private LedsState currentState = LedsState.OFF;
    private Color currentColor = Color.kBlack;
    private Animation currentAnimation = null;

    @Setter private boolean cameraDisconnected = false;

    private enum LedsState {
        OFF,
        STATIC_COLOR,
        ANIMATION
    }

    public Leds(LedsIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        LoggerUtil.recordCurrentCommand(this);

        io.updateInputs(inputs);
        Logger.processInputs("Leds", inputs);

        if (cameraDisconnected) {
            currentState = LedsState.ANIMATION;
            currentAnimation = Animations.FLASH_RED;
        }

        switch (currentState) {
            case OFF:
                io.stop();
                break;

            case STATIC_COLOR:
                io.runColor(currentColor);
                break;

            case ANIMATION:
                io.runAnimation(currentAnimation);
                break;
        }

        Logger.recordOutput("Leds/State", currentState);

        // Record cycle time
        LoggedTracer.record("Leds");
    }

    public void off() {
        currentState = LedsState.OFF;
    }

    public void runColor(Color color) {
        currentState = LedsState.STATIC_COLOR;
        currentColor = color;
    }

    public void runAnimation(Animation animation) {
        currentState = LedsState.ANIMATION;
        currentAnimation = animation;
    }
}
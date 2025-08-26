package org.frogforce503.robot2025.subsystems.leds;

import java.util.function.Consumer;

import org.frogforce503.lib.leds.Animations;

import lombok.Getter;

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

    @Getter private Consumer<LedsIO> action;

    private LedsGoal(Consumer<LedsIO> action) {
        this.action = action;
    }
}
package org.frogforce503.robot2025.subsystems.leds;

import com.ctre.phoenix.led.Animation;

public final class LedsConstants {
    public static final Animation NEUTRAL_CORAL = Animations.BREATHE_PURPLE;
    public static final Animation INTAKE_CORAL = Animations.FLASH_PURPLE;
    public static final Animation GOT_CORAL = Animations.FLASH_GREEN;
    public static final Animation SCORE_CORAL = Animations.FLASH_GREEN;

    public static final Animation NEUTRAL_ALGAE = Animations.BREATHE_BLUE;
    public static final Animation INTAKE_ALGAE = Animations.FLASH_BLUE;
    public static final Animation GOT_ALGAE = Animations.FLASH_GREEN;
    public static final Animation SCORE_ALGAE = Animations.FLASH_GREEN;

    public static final Animation USING_GLOBAL_POSE = Animations.FLASH_RED;
    public static final Animation CAMERA_DISCONNECTED = Animations.FLASH_RED;
}
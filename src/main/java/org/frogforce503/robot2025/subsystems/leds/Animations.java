package org.frogforce503.robot2025.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

/** List of commonly used animations for a CTRE CANdle. */
public final class Animations {
    // Generic animations
    private static final Animation PURPLE_STROBE =
        new StrobeAnimation(208, 89, 227, 100, 0.25, -1, 0);

    private static final Animation RED_STROBE =
        new StrobeAnimation(255, 0, 0, 0, 0.25, -1, 0);

    private static final Animation LIGHTBLUE_STROBE =
        new StrobeAnimation(173, 216, 230, 0, 0.25, -1, 0);

    private static final Animation BLUE_STROBE =
        new StrobeAnimation(0, 0, 255, 0, 0.25, -1, 0);

    private static final Animation YELLOW_STROBE =
        new StrobeAnimation(255, 255, 0, 0, 0.25, -1, 0);

    private static final Animation GREEN_STROBE =
        new StrobeAnimation(255, 0, 0, 0, 0.25, -1, 0);

    private static final Animation GREEN_FLOW =
        new ColorFlowAnimation(0, 255, 0, 0, 0.6, 45, Direction.Forward); 

    private static final Animation BREATHE_GREEN =
        new SingleFadeAnimation(0, 255, 0, 100, 0.25, -1, 0);

    private static final Animation BREATHE_PURPLE =
        new SingleFadeAnimation(255, 0, 255, 100, 0.25, -1, 0);

    private static final Animation BREATHE_BLUE =
        new SingleFadeAnimation(0, 0, 255, 100, 0.25, -1, 0);
        
    private static final Animation FLASH_GREEN =
        new SingleFadeAnimation(0, 255, 0, 100, 0.5, -1, 0);

    private static final Animation FLASH_PURPLE =
        new SingleFadeAnimation(255, 0, 255, 100, 0.5, -1, 0);

    private static final Animation FLASH_BLUE =
        new SingleFadeAnimation(0, 0, 255, 100, 0.5, -1, 0);

    private static final Animation FLASH_RED =
        new SingleFadeAnimation(255, 0, 0, 100, 0.5, -1, 0);

    // Pre-auto animations
    public static final Animation TOO_FAR_FROM_AUTO_START = Animations.RED_STROBE;
    public static final Animation NEAR_TO_AUTO_START = Animations.YELLOW_STROBE;
    public static final Animation AT_AUTO_START = Animations.GREEN_STROBE;

    // Season-specific animations
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
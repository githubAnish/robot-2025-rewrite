package org.frogforce503.robot2025.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

/** List of commonly used animations for a CTRE CANdle. */
public final class Animations {
    public static final Animation PURPLE_STROBE =
        new StrobeAnimation(208, 89, 227, 100, 0.25, -1, 0);

    public static final Animation RED_STROBE =
        new StrobeAnimation(255, 0, 0, 0, 0.25, -1, 0);

    public static final Animation LIGHTBLUE_STROBE =
        new StrobeAnimation(173, 216, 230, 0, 0.25, -1, 0);

    public static final Animation BLUE_STROBE =
        new StrobeAnimation(0, 0, 255, 0, 0.25, -1, 0);

    public static final Animation GREEN_STROBE =
        new StrobeAnimation(255, 0, 0, 0, 0.25, -1, 0);

    public static final Animation GREEN_FLOW =
        new ColorFlowAnimation(0, 255, 0, 0, 0.6, 45, Direction.Forward); 

    public static final Animation BREATHE_GREEN =
        new SingleFadeAnimation(0, 255, 0, 100, 0.25, -1, 0);

    public static final Animation BREATHE_PURPLE =
        new SingleFadeAnimation(255, 0, 255, 100, 0.25, -1, 0);

    public static final Animation BREATHE_BLUE =
        new SingleFadeAnimation(0, 0, 255, 100, 0.25, -1, 0);
        
    public static final Animation FLASH_GREEN =
        new SingleFadeAnimation(0, 255, 0, 100, 0.5, -1, 0);

    public static final Animation FLASH_PURPLE =
        new SingleFadeAnimation(255, 0, 255, 100, 0.5, -1, 0);

    public static final Animation FLASH_BLUE =
        new SingleFadeAnimation(0, 0, 255, 100, 0.5, -1, 0);

    public static final Animation FLASH_RED =
        new SingleFadeAnimation(255, 0, 0, 100, 0.5, -1, 0);
}
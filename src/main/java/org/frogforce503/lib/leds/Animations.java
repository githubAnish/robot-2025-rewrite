package org.frogforce503.lib.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import lombok.Getter;

/** List of commonly used animations for a CTRE CANdle. */
public enum Animations {
    PURPLE_STROBE(
        new StrobeAnimation(208, 89, 227, 100, 0.25, -1, 0)),

    RED_STROBE(
        new StrobeAnimation(255, 0, 0, 0, 0.25, -1, 0)),

    LIGHTBLUE_STROBE(
        new StrobeAnimation(173, 216, 230, 0, 0.25, -1, 0)),

    BLUE_STROBE(
        new StrobeAnimation(0, 0, 255, 0, 0.25, -1, 0)),

    GREEN_STROBE(
        new StrobeAnimation(255, 0, 0, 0, 0.25, -1, 0)),

    GREEN_FLOW(
        new ColorFlowAnimation(0, 255, 0, 0, 0.6, 45, Direction.Forward)), 

    BREATHE_GREEN(
        new SingleFadeAnimation(0, 255, 0, 100, 0.25, -1, 0)),

    BREATHE_PURPLE(
        new SingleFadeAnimation(255, 0, 255, 100, 0.25, -1, 0)),

    BREATHE_BLUE(
        new SingleFadeAnimation(0, 0, 255, 100, 0.25, -1, 0)),
        
    FLASH_GREEN(
        new SingleFadeAnimation(0, 255, 0, 100, 0.5, -1, 0)),

    FLASH_PURPLE(
        new SingleFadeAnimation(255, 0, 255, 100, 0.5, -1, 0)),

    FLASH_BLUE(
        new SingleFadeAnimation(0, 0, 255, 100, 0.5, -1, 0)),

    FLASH_RED(
        new SingleFadeAnimation(255, 0, 0, 100, 0.5, -1, 0));

    @Getter private Animation animation;

    private Animations(Animation animation) {
        this.animation = animation;
    }
}
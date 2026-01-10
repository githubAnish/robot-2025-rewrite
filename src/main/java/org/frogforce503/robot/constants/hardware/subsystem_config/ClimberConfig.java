package org.frogforce503.robot.constants.hardware.subsystem_config;

public record ClimberConfig(
    int id,

    boolean inverted,
    int statorCurrentLimit) {}
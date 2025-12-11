package org.frogforce503.robot2025.constants.hardware.subsystem_config;

public record ClimberConfig(
    int id,

    boolean inverted,
    int statorCurrentLimit) {}
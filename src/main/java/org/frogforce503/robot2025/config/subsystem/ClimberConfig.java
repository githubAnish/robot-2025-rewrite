package org.frogforce503.robot2025.config.subsystem;

public record ClimberConfig(
    int id,

    boolean inverted,
    int statorCurrentLimit) {}
package org.frogforce503.robot2025.config.subsystem;

import lombok.Builder;

@Builder
public record ClimberConfig(
    int winchID,
    boolean winchInverted) {}
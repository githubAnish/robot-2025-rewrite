package org.frogforce503.robot2025.constants.configs;

import lombok.Builder;

@Builder
public record ClimberConfig(
    int winchID,
    boolean winchInverted) {}
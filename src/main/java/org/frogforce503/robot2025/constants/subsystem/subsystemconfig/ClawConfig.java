package org.frogforce503.robot2025.constants.subsystem.subsystemconfig;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

public record ClawConfig(
    int leftId,
    int rightId,
    double mechanismRatio,

    boolean leftInverted,
    boolean rightInverted,
    int statorCurrentLimit,
    
    PIDConfig kPID,
    FFConfig kFF) {}
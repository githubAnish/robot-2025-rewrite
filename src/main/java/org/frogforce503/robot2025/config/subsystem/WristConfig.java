package org.frogforce503.robot2025.config.subsystem;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

public record WristConfig(
    int id,
    double mechanismRatio,

    boolean inverted,
    int statorCurrentLimit,
    double zeroOffset,

    PIDConfig kPID,
    FFConfig kFF,
    Range motionRange) {}
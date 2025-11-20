package org.frogforce503.robot2025.config.subsystem;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import lombok.Builder;

@Builder
public record WristConfig(
    int wristID,
    boolean wristInverted,
    double wristOffset,
    PIDConfig kPID,
    FFConfig kFF,
    Range range,
    double horizontalAngle) {}
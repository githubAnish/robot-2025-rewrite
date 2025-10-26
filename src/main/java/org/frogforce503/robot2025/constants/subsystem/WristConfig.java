package org.frogforce503.robot2025.constants.subsystem;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;

import lombok.Builder;

@Builder
public record WristConfig(
    int wristID,
    boolean wristInverted,
    double wristOffset,
    PIDFConfig kPIDF,
    Range range) {}